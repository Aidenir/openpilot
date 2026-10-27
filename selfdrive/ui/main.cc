#include <execinfo.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

#include <QApplication>
#include <QTranslator>

#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/window.h"

// FrogPilot variables
// The manager's watchdog sends SIGUSR1 just before it kills a frozen UI: the main thread writes where it is stuck, a backtrace
// and the memory map to resolve it against, to /tmp/ui_backtrace_<pid>.txt, which goes into the watchdog dump. Its freezes
// (2026-10) all stop after a finished paint, and the dump is taken after the kill, so without this nothing says where
static pid_t main_tid;
static char backtrace_path[64];

static void write_all(int fd, const char *buf, ssize_t len) {
  while (len > 0) {
    ssize_t n = write(fd, buf, len);
    if (n <= 0) return;
    buf += n;
    len -= n;
  }
}

static void backtrace_handler(int) {
  if (syscall(SYS_gettid) != main_tid) {
    syscall(SYS_tgkill, getpid(), main_tid, SIGUSR1);
    return;
  }
  int fd = open(backtrace_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) return;
  void *frames[64];
  int count = backtrace(frames, 64);
  backtrace_symbols_fd(frames, count, fd);

  const char header[] = "\n/proc/self/maps:\n";
  write_all(fd, header, sizeof(header) - 1);
  int maps = open("/proc/self/maps", O_RDONLY);
  if (maps >= 0) {
    char buf[4096];
    ssize_t n;
    while ((n = read(maps, buf, sizeof(buf))) > 0) write_all(fd, buf, n);
    close(maps);
  }
  close(fd);
}

static void install_backtrace_handler() {
  main_tid = syscall(SYS_gettid);
  snprintf(backtrace_path, sizeof(backtrace_path), "/tmp/ui_backtrace_%d.txt", getpid());
  void *frame;
  backtrace(&frame, 1);  // loads libgcc now, backtrace() may allocate the first time and the handler must not

  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = backtrace_handler;
  sa.sa_flags = SA_RESTART;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGUSR1, &sa, nullptr);
}

int main(int argc, char *argv[]) {
  setpriority(PRIO_PROCESS, 0, -20);
  install_backtrace_handler();

  qInstallMessageHandler(swagLogMessageHandler);
  initApp(argc, argv);

  QTranslator translator;
  QString translation_file = QString::fromStdString(Params().get("LanguageSetting"));
  if (!translator.load(QString(":/%1").arg(translation_file)) && translation_file.length()) {
    qCritical() << "Failed to load translation file:" << translation_file;
  }

  QApplication a(argc, argv);
  a.installTranslator(&translator);

  MainWindow w;
  setMainWindow(&w);
  a.installEventFilter(&w);
  return a.exec();
}
