import importlib
import os
import signal
import shutil
import struct
import threading
import time
import subprocess
from collections.abc import Callable, ValuesView
from abc import ABC, abstractmethod
from multiprocessing import Process
from types import SimpleNamespace

from setproctitle import setproctitle

from cereal import car, log
import cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.watchdog import WATCHDOG_FN

ENABLE_WATCHDOG = os.getenv("NO_WATCHDOG") is None
WATCHDOG_DIAG_DIR = "/data/error_logs/watchdog"
WATCHDOG_DIAG_MAX_READ = 1024 * 1024
WATCHDOG_SENTRY_MAX_ATTACHMENT_BYTES = 512 * 1024

WATCHDOG_PROC_FILES = (
  "cmdline", "comm", "cgroup", "io", "limits", "maps", "mountinfo", "oom_score", "oom_score_adj",
  "sched", "schedstat", "smaps_rollup", "stack", "stat", "statm", "status", "syscall", "wchan",
)
WATCHDOG_THREAD_FILES = ("children", "comm", "sched", "schedstat", "stack", "stat", "statm", "status", "syscall", "wchan")
WATCHDOG_SYSTEM_FILES = (
  "/proc/loadavg", "/proc/meminfo", "/proc/pressure/cpu", "/proc/pressure/io", "/proc/pressure/memory",
  "/proc/buddyinfo", "/proc/diskstats", "/proc/interrupts", "/proc/sched_debug", "/proc/schedstat",
  "/proc/softirqs", "/proc/stat", "/proc/swaps", "/proc/uptime", "/proc/vmstat", "/proc/zoneinfo",
  "/sys/class/kgsl/kgsl-3d0/devfreq/cur_freq", "/sys/class/kgsl/kgsl-3d0/gpu_available_frequencies",
  "/sys/class/kgsl/kgsl-3d0/gpubusy", "/sys/class/kgsl/kgsl-3d0/gpuclk", "/sys/class/kgsl/kgsl-3d0/gpu_model",
  "/sys/class/kgsl/kgsl-3d0/idle_timer", "/sys/class/kgsl/kgsl-3d0/max_gpuclk",
  "/sys/class/kgsl/kgsl-3d0/min_gpuclk", "/sys/class/kgsl/kgsl-3d0/reset_count",
  "/sys/class/kgsl/kgsl-3d0/throttling", "/sys/class/kgsl/kgsl-3d0/thermal_pwrlevel",
  "/sys/kernel/debug/dri/0/clients", "/sys/kernel/debug/dri/0/state",
)
WATCHDOG_SENTRY_ATTACHMENTS = (
  "summary.txt",
  "sentry_thread_proc.txt",
  "proc/{pid}/watchdog_stage.txt",
  "proc/{pid}/status",
  "proc/{pid}/wchan",
  "proc/{pid}/stack",
  "proc/{pid}/smaps_rollup",
  "commands/gdb_thread_backtraces.txt",
  "commands/kgsl_debugfs_snapshot.txt",
  "commands/drm_debugfs_snapshot.txt",
  "commands/ipc_shm_listing.txt",
  "commands/ps_ui_threads.txt",
  "commands/ps_ui_children.txt",
  "commands/ps_weston_threads.txt",
  "commands/ps_related_processes.txt",
  "commands/ps_top_cpu.txt",
  "commands/ps_top_mem.txt",
  "commands/dmesg.txt",
  "commands/journalctl_boot_tail.txt",
  "commands/logcat_tail.txt",
)


def launcher(proc: str, name: str) -> None:
  try:
    # import the process
    mod = importlib.import_module(proc)

    # rename the process
    setproctitle(proc)

    # create new context since we forked
    messaging.reset_context()

    # add daemon name tag to logs
    cloudlog.bind(daemon=name)
    sentry.set_tag("daemon", name)

    # exec the process
    mod.main()
  except KeyboardInterrupt:
    cloudlog.warning(f"child {proc} got SIGINT")
  except Exception:
    # can't install the crash handler because sys.excepthook doesn't play nice
    # with threads, so catch it here.
    sentry.capture_exception()
    raise


def nativelauncher(pargs: list[str], cwd: str, name: str) -> None:
  os.environ['MANAGER_DAEMON'] = name

  # exec the process
  os.chdir(cwd)
  os.execvp(pargs[0], pargs)


def join_process(process: Process, timeout: float) -> None:
  # Process().join(timeout) will hang due to a python 3 bug: https://bugs.python.org/issue28382
  # We have to poll the exitcode instead
  t = time.monotonic()
  while time.monotonic() - t < timeout and process.exitcode is None:
    time.sleep(0.001)


class ManagerProcess(ABC):
  daemon = False
  sigkill = False
  should_run: Callable[[bool, Params, car.CarParams, SimpleNamespace], bool]
  proc: Process | None = None
  enabled = True
  name = ""

  last_watchdog_time = 0
  watchdog_max_dt: int | None = None
  watchdog_seen = False
  shutting_down = False

  @abstractmethod
  def prepare(self) -> None:
    pass

  @abstractmethod
  def start(self) -> None:
    pass

  def restart(self) -> None:
    self.stop(sig=signal.SIGKILL)
    self.start()

  @staticmethod
  def _read_diag_file(path: str, max_bytes: int = WATCHDOG_DIAG_MAX_READ) -> str:
    with open(path, "rb") as f:
      data = f.read(max_bytes + 1)
    truncated = len(data) > max_bytes
    text = data[:max_bytes].decode("utf-8", errors="replace").replace("\x00", "\\0")
    if truncated:
      text += f"\n<truncated after {max_bytes} bytes>\n"
    return text

  @staticmethod
  def _write_diag_file(path: str, contents: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
      f.write(contents)
      if not contents.endswith("\n"):
        f.write("\n")

  @staticmethod
  def _read_diag_attachment(path: str, max_bytes: int = WATCHDOG_SENTRY_MAX_ATTACHMENT_BYTES) -> bytes:
    with open(path, "rb") as f:
      data = f.read(max_bytes + 1)
    if len(data) <= max_bytes:
      return data
    marker = f"\n<truncated after {max_bytes} bytes>\n".encode("utf-8")
    return data[:max_bytes] + marker

  def _copy_diag_file(self, src: str, dst: str, max_bytes: int = WATCHDOG_DIAG_MAX_READ) -> None:
    try:
      self._write_diag_file(dst, self._read_diag_file(src, max_bytes=max_bytes))
    except Exception as e:
      self._write_diag_file(dst, f"unavailable: {e!r}")

  def _run_watchdog_diag_cmd(self, diag_dir: str, name: str, cmd: list[str], timeout: float = 2.0, max_bytes: int = WATCHDOG_DIAG_MAX_READ) -> None:
    dst = os.path.join(diag_dir, "commands", f"{name}.txt")
    exe = shutil.which(cmd[0])
    if exe is None:
      self._write_diag_file(dst, f"command not found: {cmd[0]}")
      return

    try:
      result = subprocess.run(
        [exe, *cmd[1:]],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
      )
      output = result.stdout[:max_bytes].decode("utf-8", errors="replace")
      if len(result.stdout) > max_bytes:
        output += f"\n<truncated after {max_bytes} bytes>\n"
      self._write_diag_file(dst, f"$ {' '.join(cmd)}\nexit={result.returncode}\n\n{output}")
    except subprocess.TimeoutExpired as e:
      output = ""
      if e.stdout:
        output = e.stdout[:max_bytes].decode("utf-8", errors="replace")
      self._write_diag_file(dst, f"$ {' '.join(cmd)}\ntimed out after {e.timeout}s\n\n{output}")
    except Exception as e:
      self._write_diag_file(dst, f"$ {' '.join(cmd)}\nfailed: {e!r}")

  def _dump_watchdog_proc_tree(self, diag_dir: str, pid: int) -> None:
    proc_dir = os.path.join(diag_dir, "proc", str(pid))
    self._copy_diag_file(f"{WATCHDOG_FN}stage_{pid}", os.path.join(proc_dir, "watchdog_stage.txt"), max_bytes=64 * 1024)
    for proc_file in WATCHDOG_PROC_FILES:
      self._copy_diag_file(f"/proc/{pid}/{proc_file}", os.path.join(proc_dir, proc_file))

    fd_lines = []
    try:
      for fd in sorted(os.listdir(f"/proc/{pid}/fd"), key=lambda x: int(x) if x.isdigit() else x):
        fd_path = f"/proc/{pid}/fd/{fd}"
        try:
          fd_lines.append(f"{fd} -> {os.readlink(fd_path)}")
        except Exception as e:
          fd_lines.append(f"{fd} -> unavailable: {e!r}")
    except Exception as e:
      fd_lines.append(f"unavailable: {e!r}")
    self._write_diag_file(os.path.join(proc_dir, "fd.txt"), "\n".join(fd_lines))

    try:
      tids = sorted(os.listdir(f"/proc/{pid}/task"), key=lambda x: int(x) if x.isdigit() else x)
    except Exception as e:
      self._write_diag_file(os.path.join(proc_dir, "task_error.txt"), f"unavailable: {e!r}")
      return

    for tid in tids:
      task_dir = os.path.join(proc_dir, "task", tid)
      for proc_file in WATCHDOG_THREAD_FILES:
        self._copy_diag_file(f"/proc/{pid}/task/{tid}/{proc_file}", os.path.join(task_dir, proc_file))

  def _dump_watchdog_system_state(self, diag_dir: str, pid: int) -> None:
    for src in WATCHDOG_SYSTEM_FILES:
      dst = os.path.join(diag_dir, "system", src.lstrip("/").replace("/", "__"))
      self._copy_diag_file(src, dst)

    commands = (
      ("ps_ui_threads", ["ps", "-T", "-p", str(pid), "-o", "pid,tid,comm,stat,pcpu,pmem,wchan:32,etime,pri,ni"]),
      ("ps_ui_children", ["sh", "-c", f"for p in $(pgrep -P {pid}); do ps -T -p $p -o pid,ppid,tid,comm,stat,pcpu,pmem,wchan:32,etime,pri,ni,args; done"]),
      ("ps_weston_threads", ["sh", "-c", "for p in $(pgrep -x weston); do ps -T -p $p -o pid,ppid,tid,comm,stat,pcpu,pmem,wchan:32,etime,pri,ni,args; done"]),
      ("ps_related_processes", ["sh", "-c", "ps -eo pid,ppid,tid,stat,pcpu,pmem,wchan:32,comm,args | grep -E '(^ *PID| weston|\\./ui|logmessaged|hardwared|statsd|frogpilot|mapd|pandad|loggerd|timed|tombstoned)' | grep -v grep"]),
      ("ps_top_cpu", ["ps", "-eo", "pid,ppid,stat,pcpu,pmem,wchan:32,comm,args", "--sort=-pcpu"]),
      ("ps_top_mem", ["ps", "-eo", "pid,ppid,stat,pcpu,pmem,wchan:32,comm,args", "--sort=-pmem"]),
      ("ipc_shm_listing", ["sh", "-c", "ls -la /dev/shm /data/shm /tmp 2>&1 | head -300"]),
      ("kgsl_debugfs_snapshot", ["sh", "-c", "for f in /sys/kernel/debug/kgsl/kgsl-3d0/* /sys/kernel/debug/kgsl/proc/*/*; do [ -f \"$f\" ] && [ -r \"$f\" ] && echo \"===== $f =====\" && head -c 65536 \"$f\" && echo; done 2>&1"], 2.0, WATCHDOG_DIAG_MAX_READ),
      ("drm_debugfs_snapshot", ["sh", "-c", "for f in /sys/kernel/debug/dri/0/*; do [ -f \"$f\" ] && [ -r \"$f\" ] && echo \"===== $f =====\" && head -c 65536 \"$f\" && echo; done 2>&1"], 2.0, WATCHDOG_DIAG_MAX_READ),
      ("dmesg", ["dmesg"], 2.0, 2 * WATCHDOG_DIAG_MAX_READ),
      ("journalctl_boot_tail", ["journalctl", "-b", "-o", "short-precise", "--no-pager", "-n", "1200"], 3.0, 2 * WATCHDOG_DIAG_MAX_READ),
      ("logcat_tail", ["logcat", "-d", "-v", "threadtime", "-t", "3000"], 3.0, 2 * WATCHDOG_DIAG_MAX_READ),
    )

    for command in commands:
      name, cmd, *opts = command
      timeout = opts[0] if len(opts) > 0 else 2.0
      max_bytes = opts[1] if len(opts) > 1 else WATCHDOG_DIAG_MAX_READ
      self._run_watchdog_diag_cmd(diag_dir, name, cmd, timeout=timeout, max_bytes=max_bytes)

  def _dump_watchdog_gdb(self, diag_dir: str, pid: int) -> None:
    gdb = shutil.which("gdb")
    dst = os.path.join(diag_dir, "commands", "gdb_thread_backtraces.txt")
    if gdb is None:
      self._write_diag_file(dst, "gdb not found")
      return

    try:
      result = subprocess.run(
        [
          gdb, "-batch", "-nx",
          "-iex", "set debuginfod enabled off",
          "-iex", "set auto-load safe-path /",
          "-p", str(pid),
          "-ex", "set pagination off",
          "-ex", "set confirm off",
          "-ex", "set print elements 64",
          "-ex", "set print repeats 8",
          "-ex", "set print pretty off",
          "-ex", "info threads",
          "-ex", "thread apply all bt 80",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=30,
      )
      self._write_diag_file(dst, result.stdout.decode("utf-8", errors="replace"))
    except subprocess.TimeoutExpired as e:
      output = ""
      if e.stdout:
        output = e.stdout.decode("utf-8", errors="replace")
      self._write_diag_file(dst, f"timed out after {e.timeout}s\n\n{output}")
    except Exception as e:
      self._write_diag_file(dst, f"failed: {e!r}")

  def _write_watchdog_sentry_thread_report(self, diag_dir: str, pid: int) -> str:
    task_base = os.path.join(diag_dir, "proc", str(pid), "task")
    report_path = os.path.join(diag_dir, "sentry_thread_proc.txt")
    lines = []

    try:
      tids = sorted(os.listdir(task_base), key=lambda x: int(x) if x.isdigit() else x)
    except Exception as e:
      self._write_diag_file(report_path, f"unavailable: {e!r}")
      return report_path

    for tid in tids:
      lines.append(f"\n===== tid {tid} =====")
      for proc_file in ("comm", "status", "wchan", "syscall", "stack", "sched"):
        path = os.path.join(task_base, tid, proc_file)
        lines.append(f"\n--- {proc_file} ---")
        try:
          lines.append(self._read_diag_file(path, max_bytes=64 * 1024))
        except Exception as e:
          lines.append(f"unavailable: {e!r}")

    report = "\n".join(lines)
    encoded = report.encode("utf-8", errors="replace")
    if len(encoded) > WATCHDOG_SENTRY_MAX_ATTACHMENT_BYTES:
      report = encoded[:WATCHDOG_SENTRY_MAX_ATTACHMENT_BYTES].decode("utf-8", errors="replace")
      report += f"\n<truncated after {WATCHDOG_SENTRY_MAX_ATTACHMENT_BYTES} bytes>\n"
    self._write_diag_file(report_path, report)
    return report_path

  def _report_watchdog_diagnostics_to_sentry(self, diag_dir: str, pid: int, dt: float) -> None:
    try:
      self._write_watchdog_sentry_thread_report(diag_dir, pid)

      attachments = []
      for rel_path in WATCHDOG_SENTRY_ATTACHMENTS:
        rel_path = rel_path.format(pid=pid)
        path = os.path.join(diag_dir, rel_path)
        try:
          filename = rel_path.replace("/", "__")
          attachments.append((filename, self._read_diag_attachment(path), "text/plain"))
        except Exception as e:
          attachments.append((rel_path.replace("/", "__") + ".missing.txt", f"unavailable: {e!r}", "text/plain"))

      sentry.capture_message(
        message=f"UI watchdog timeout diagnostics captured ({dt:.3f}s)",
        level="error",
        extras={
          "watchdog_diag_dir": diag_dir,
          "watchdog_pid": pid,
          "watchdog_dt": f"{dt:.3f}",
          "watchdog_max_dt": self.watchdog_max_dt,
          "last_watchdog_time": self.last_watchdog_time,
        },
        tags={
          "watchdog_daemon": self.name,
          "watchdog_reason": "timeout",
        },
        attachments=attachments,
      )
    except Exception:
      cloudlog.exception(f"failed to send watchdog diagnostics for {self.name} to sentry")

  def dump_watchdog_diagnostics(self, dt: float, pid: int | None = None, exitcode: int | None = None, gdb: bool = True) -> None:
    if pid is None:
      if self.proc is None or self.proc.pid is None:
        return
      pid, exitcode = self.proc.pid, self.proc.exitcode
    if self.name != "ui":
      return

    try:
      os.makedirs(WATCHDOG_DIAG_DIR, exist_ok=True)
      ts = time.strftime("%Y-%m-%d--%H-%M-%S")
      diag_dir = os.path.join(WATCHDOG_DIAG_DIR, f"{ts}--{self.name}--pid-{pid}")

      self._write_diag_file(os.path.join(diag_dir, "summary.txt"), "\n".join((
        f"name={self.name}",
        f"pid={pid}",
        f"exitcode={exitcode}",
        f"started={exitcode is None}",
        f"gdb={gdb}",
        f"watchdog_dt={dt:.3f}s",
        f"watchdog_max_dt={self.watchdog_max_dt}s",
        f"last_watchdog_time={self.last_watchdog_time}",
        f"manager_monotonic={time.monotonic():.6f}",
        f"wall_time={time.strftime('%Y-%m-%d %H:%M:%S %Z')}",
      )))

      self._dump_watchdog_proc_tree(diag_dir, pid)
      self._dump_watchdog_system_state(diag_dir, pid)
      if gdb:
        self._dump_watchdog_gdb(diag_dir, pid)
      self._report_watchdog_diagnostics_to_sentry(diag_dir, pid, dt)
      self._write_diag_file(os.path.join(diag_dir, "DONE"), "complete")

      cloudlog.error(f"Watchdog diagnostics for {self.name} written to {diag_dir}")
    except Exception:
      cloudlog.exception(f"failed to dump watchdog diagnostics for {self.name}")

  def check_watchdog(self, started: bool) -> None:
    if self.watchdog_max_dt is None or self.proc is None:
      return

    try:
      fn = WATCHDOG_FN + str(self.proc.pid)
      with open(fn, "rb") as f:
        self.last_watchdog_time = struct.unpack('Q', f.read())[0]
    except Exception:
      pass

    dt = time.monotonic() - self.last_watchdog_time / 1e9

    if dt > self.watchdog_max_dt:
      if self.watchdog_seen and ENABLE_WATCHDOG:
        cloudlog.error(f"Watchdog timeout for {self.name} (exitcode {self.proc.exitcode}) restarting ({started=})")
        if started:
          # FrogPilot: onroad, restart straight away and collect the diagnostics in the background without gdb. Done in line they
          # took ~8 s (gdb alone may take 30), the UI stayed frozen all that time and the manager stopped publishing managerState,
          # so selfdrived raised commIssue on top of the frozen screen (2026-10-02)
          pid, exitcode = self.proc.pid, self.proc.exitcode
          self.restart()
          threading.Thread(target=self.dump_watchdog_diagnostics, args=(dt, pid, exitcode, False), daemon=True).start()
        else:
          self.dump_watchdog_diagnostics(dt)
          self.restart()
    else:
      self.watchdog_seen = True

  def stop(self, retry: bool = True, block: bool = True, sig: signal.Signals = None) -> int | None:
    if self.proc is None:
      return None

    if self.proc.exitcode is None:
      if not self.shutting_down:
        cloudlog.info(f"killing {self.name}")
        if sig is None:
          sig = signal.SIGKILL if self.sigkill else signal.SIGINT
        self.signal(sig)
        self.shutting_down = True

        if not block:
          return None

      join_process(self.proc, 5)

      # If process failed to die send SIGKILL
      if self.proc.exitcode is None and retry:
        cloudlog.info(f"killing {self.name} with SIGKILL")
        self.signal(signal.SIGKILL)
        self.proc.join()

    ret = self.proc.exitcode
    cloudlog.info(f"{self.name} is dead with {ret}")

    if self.proc.exitcode is not None:
      self.shutting_down = False
      self.proc = None

    return ret

  def signal(self, sig: int) -> None:
    if self.proc is None:
      return

    # Don't signal if already exited
    if self.proc.exitcode is not None and self.proc.pid is not None:
      return

    # Can't signal if we don't have a pid
    if self.proc.pid is None:
      return

    cloudlog.info(f"sending signal {sig} to {self.name}")
    os.kill(self.proc.pid, sig)

  def get_process_state_msg(self):
    state = log.ManagerState.ProcessState.new_message()
    state.name = self.name
    if self.proc:
      state.running = self.proc.is_alive()
      state.shouldBeRunning = self.proc is not None and not self.shutting_down
      state.pid = self.proc.pid or 0
      state.exitCode = self.proc.exitcode or 0
    return state


class NativeProcess(ManagerProcess):
  def __init__(self, name, cwd, cmdline, should_run, enabled=True, sigkill=False, watchdog_max_dt=None):
    self.name = name
    self.cwd = cwd
    self.cmdline = cmdline
    self.should_run = should_run
    self.enabled = enabled
    self.sigkill = sigkill
    self.watchdog_max_dt = watchdog_max_dt
    self.launcher = nativelauncher

  def prepare(self) -> None:
    pass

  def start(self) -> None:
    # In case we only tried a non blocking stop we need to stop it before restarting
    if self.shutting_down:
      self.stop()

    if self.proc is not None:
      return

    cwd = os.path.join(BASEDIR, self.cwd)
    cloudlog.info(f"starting process {self.name}")
    self.proc = Process(name=self.name, target=self.launcher, args=(self.cmdline, cwd, self.name))
    self.proc.start()
    self.watchdog_seen = False
    self.shutting_down = False


class PythonProcess(ManagerProcess):
  def __init__(self, name, module, should_run, enabled=True, sigkill=False, watchdog_max_dt=None):
    self.name = name
    self.module = module
    self.should_run = should_run
    self.enabled = enabled
    self.sigkill = sigkill
    self.watchdog_max_dt = watchdog_max_dt
    self.launcher = launcher

  def prepare(self) -> None:
    if self.enabled:
      cloudlog.info(f"preimporting {self.module}")
      importlib.import_module(self.module)

  def start(self) -> None:
    # In case we only tried a non blocking stop we need to stop it before restarting
    if self.shutting_down:
      self.stop()

    if self.proc is not None:
      return

    # TODO: this is just a workaround for this tinygrad check:
    # https://github.com/tinygrad/tinygrad/blob/ac9c96dae1656dc220ee4acc39cef4dd449aa850/tinygrad/device.py#L26
    name = self.name if "modeld" not in self.name else "MainProcess"

    cloudlog.info(f"starting python {self.module}")
    self.proc = Process(name=name, target=self.launcher, args=(self.module, self.name))
    self.proc.start()
    self.watchdog_seen = False
    self.shutting_down = False


class DaemonProcess(ManagerProcess):
  """Python process that has to stay running across manager restart.
  This is used for athena so you don't lose SSH access when restarting manager."""
  def __init__(self, name, module, param_name, enabled=True):
    self.name = name
    self.module = module
    self.param_name = param_name
    self.enabled = enabled
    self.params = None

  @staticmethod
  def should_run(started, params, CP, frogpilot_toggles):
    return True

  def prepare(self) -> None:
    pass

  def start(self) -> None:
    if self.params is None:
      self.params = Params()

    pid = self.params.get(self.param_name)
    if pid is not None:
      try:
        os.kill(int(pid), 0)
        with open(f'/proc/{pid}/cmdline') as f:
          if self.module in f.read():
            # daemon is running
            return
      except (OSError, FileNotFoundError):
        # process is dead
        pass

    cloudlog.info(f"starting daemon {self.name}")
    proc = subprocess.Popen(['python', '-m', self.module],
                               stdin=open('/dev/null'),
                               stdout=open('/dev/null', 'w'),
                               stderr=open('/dev/null', 'w'),
                               preexec_fn=os.setpgrp)

    self.params.put(self.param_name, proc.pid)

  def stop(self, retry=True, block=True, sig=None) -> None:
    pass


def ensure_running(procs: ValuesView[ManagerProcess], started: bool, params=None, CP: car.CarParams=None,
                   not_run: list[str] | None=None, frogpilot_toggles: SimpleNamespace=None) -> list[ManagerProcess]:
  if not_run is None:
    not_run = []

  running = []
  for p in procs:
    if p.enabled and p.name not in not_run and p.should_run(started, params, CP, frogpilot_toggles):
      running.append(p)
    else:
      p.stop(block=False)

    p.check_watchdog(started)

  for p in running:
    p.start()

  return running
