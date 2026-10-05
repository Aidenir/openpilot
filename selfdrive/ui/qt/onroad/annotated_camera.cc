
#include "selfdrive/ui/qt/onroad/annotated_camera.h"

#include <QPainter>
#include <algorithm>
#include <cmath>

#include "common/swaglog.h"
#include "common/watchdog.h"
#include "selfdrive/ui/qt/util.h"

// Window that shows camera view and variety of info drawn on top
AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget *parent)
    : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraWidget("camerad", type, parent) {
  pm = std::make_unique<PubMaster>(std::vector<const char*>{"uiDebug"});

  main_layout = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  main_layout->setSpacing(0);

  experimental_btn = new ExperimentalButton(this);
  main_layout->addWidget(experimental_btn, 0, Qt::AlignTop | Qt::AlignRight);

  // FrogPilot variables
  personality_btn = new DrivingPersonalityButton(this);
  personality_btn->setVisible(false);

  speed_bump_mark_btn = new SpeedBumpMarkButton(this);
  speed_bump_mark_btn->setVisible(false);

  refuge_island_mark_btn = new RefugeIslandMarkButton(this);
  refuge_island_mark_btn->setVisible(false);

  gps_reboot_btn = new GpsRebootButton(this);
  gps_reboot_btn->setVisible(false);

  screen_recorder = new ScreenRecorder(this);
  screen_recorder->setVisible(false);
}

void AnnotatedCameraWidget::updateState(const UIState &s, const FrogPilotUIState &fs) {
  // update engageability/experimental mode button
  experimental_btn->updateState(s, fs);
  dmon.updateState(s);

  // FrogPilot variables
  const SubMaster &sm = *(s.sm);

  const cereal::CarState::Reader &carState = sm["carState"].getCarState();

  frogpilot_nvg->experimentalButtonPosition = QPoint(experimental_btn->x(), experimental_btn->y());

  bool onroad_distance_btn_enabled = frogpilot_nvg->dmIconPosition != QPoint(0, 0) && !frogpilot_nvg->hideBottomIcons && frogpilot_toggles.value("onroad_distance_button").toBool();
  personality_btn->setVisible(onroad_distance_btn_enabled);
  if (onroad_distance_btn_enabled) {
    personality_btn->move(frogpilot_nvg->rightHandDM ? width() - UI_BORDER_SIZE - personality_btn->width() - (UI_BORDER_SIZE / 2) : UI_BORDER_SIZE, frogpilot_nvg->dmIconPosition.y() - personality_btn->height() / 2);
    personality_btn->updateState(s, fs);
  }

  dmon.onroad_distance_btn_enabled = onroad_distance_btn_enabled;

  // On the driver's side, a little below the middle: clear of the set speed and speed
  // limit sign at the top and the driver monitoring, compass and weather icons below
  bool speed_bump_mark_btn_enabled = frogpilot_toggles.value("user_speed_bump_button").toBool();
  speed_bump_mark_btn->setVisible(speed_bump_mark_btn_enabled);
  if (speed_bump_mark_btn_enabled) {
    int x = frogpilot_nvg->rightHandDM ? width() - UI_BORDER_SIZE * 2 - speed_bump_mark_btn->width() : UI_BORDER_SIZE * 2;
    speed_bump_mark_btn->move(x, height() * 0.55 - speed_bump_mark_btn->height() / 2);
    speed_bump_mark_btn->updateState(frogpilot_toggles.value("speed_bump_detect").toBool());
  }

  // Beside the speed bump button, further in, or in its place when that is off. Stacked under it, it ran into the onroad
  // distance button and the driver monitoring icon below
  bool refuge_island_mark_btn_enabled = frogpilot_toggles.value("user_refuge_island_button").toBool();
  refuge_island_mark_btn->setVisible(refuge_island_mark_btn_enabled);
  if (refuge_island_mark_btn_enabled) {
    int x = frogpilot_nvg->rightHandDM ? width() - UI_BORDER_SIZE * 2 - refuge_island_mark_btn->width() : UI_BORDER_SIZE * 2;
    int y = height() * 0.55 - speed_bump_mark_btn->height() / 2;
    if (speed_bump_mark_btn_enabled) {
      int step = speed_bump_mark_btn->width() + UI_BORDER_SIZE;
      x += frogpilot_nvg->rightHandDM ? -step : step;
    }
    refuge_island_mark_btn->move(x, y);
    refuge_island_mark_btn->updateState();
  }

  // So the approach icons can be drawn clear of the mark buttons
  QRect mark_buttons;
  if (speed_bump_mark_btn_enabled) {
    mark_buttons |= speed_bump_mark_btn->geometry();
  }
  if (refuge_island_mark_btn_enabled) {
    mark_buttons |= refuge_island_mark_btn->geometry();
  }
  frogpilot_nvg->markButtonsRect = mark_buttons;

  // Across from the speed bump button, on the passenger's side
  bool gps_reboot_btn_enabled = GpsRebootButton::alertShown(s);
  gps_reboot_btn->updateState(s);
  gps_reboot_btn->setVisible(gps_reboot_btn_enabled);
  if (gps_reboot_btn_enabled) {
    int x = frogpilot_nvg->rightHandDM ? UI_BORDER_SIZE * 2 : width() - UI_BORDER_SIZE * 2 - gps_reboot_btn->width();
    gps_reboot_btn->move(x, height() * 0.55 - gps_reboot_btn->height() / 2);
  }

  screen_recorder->move(experimental_btn->x() - UI_BORDER_SIZE - btn_size, experimental_btn->y());
  screen_recorder->setVisible(frogpilot_nvg->standstillDuration == 0 && !(frogpilot_nvg->signalStyle == "static" && carState.getRightBlinker()) && frogpilot_toggles.value("screen_recorder").toBool());
}

void AnnotatedCameraWidget::initializeGL() {
  CameraWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

mat4 AnnotatedCameraWidget::calcFrameMatrix() {
  // Project point at "infinity" to compute x and y offsets
  // to ensure this ends up in the middle of the screen
  // for narrow come and a little lower for wide cam.
  // TODO: use proper perspective transform?

  // Select intrinsic matrix and calibration based on camera type
  auto *s = uiState();
  bool wide_cam = active_stream_type == VISION_STREAM_WIDE_ROAD;
  const auto &intrinsic_matrix = wide_cam ? ECAM_INTRINSIC_MATRIX : FCAM_INTRINSIC_MATRIX;
  const auto &calibration = wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;

   // Compute the calibration transformation matrix
  const auto calib_transform = intrinsic_matrix * calibration;

  float zoom = wide_cam ? 2.0 : 1.1;
  Eigen::Vector3f inf(1000., 0., 0.);
  auto Kep = calib_transform * inf;

  int w = width(), h = height();
  float center_x = intrinsic_matrix(0, 2);
  float center_y = intrinsic_matrix(1, 2);

  float max_x_offset = center_x * zoom - w / 2 - 5;
  float max_y_offset = center_y * zoom - h / 2 - 5;
  float x_offset = std::clamp<float>((Kep.x() / Kep.z() - center_x) * zoom, -max_x_offset, max_x_offset);
  float y_offset = std::clamp<float>((Kep.y() / Kep.z() - center_y) * zoom, -max_y_offset, max_y_offset);

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  Eigen::Matrix3f video_transform =(Eigen::Matrix3f() <<
    zoom, 0.0f, (w / 2 - x_offset) - (center_x * zoom),
    0.0f, zoom, (h / 2 - y_offset) - (center_y * zoom),
    0.0f, 0.0f, 1.0f).finished();

  model.setTransform(video_transform * calib_transform);

  float zx = zoom * 2 * center_x / w;
  float zy = zoom * 2 * center_y / h;
  return mat4{{
    zx, 0.0, 0.0, -x_offset / w * 2,
    0.0, zy, 0.0, y_offset / h * 2,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
  }};
}

void AnnotatedCameraWidget::paintGL() {
}

void AnnotatedCameraWidget::paintEvent(QPaintEvent *event) {
  UIState *s = uiState();
  SubMaster &sm = *(s->sm);
  const double start_draw_t = millis_since_boot();

  watchdog_stage("paint:start");
  QPainter painter(this);
  watchdog_stage("paint:after_qpainter");

  // draw camera frame
  {
    watchdog_stage("paint:before_frame_lock");
    std::lock_guard lk(frame_lock);
    watchdog_stage("paint:after_frame_lock");

    if (frames.empty()) {
      if (skip_frame_count > 0) {
        skip_frame_count--;
        qDebug() << "skipping frame, not ready";
        watchdog_stage("paint:return_frame_not_ready");
        return;
      }
    } else {
      // skip drawing up to this many frames if we're
      // missing camera frames. this smooths out the
      // transitions from the narrow and wide cameras
      skip_frame_count = 5;
    }

    // Wide or narrow cam dependent on speed
    bool has_wide_cam = available_streams.count(VISION_STREAM_WIDE_ROAD);
    if (has_wide_cam) {
      float v_ego = sm["carState"].getCarState().getVEgo();
      if ((v_ego < 10) || available_streams.size() == 1) {
        wide_cam_requested = true;
      } else if (v_ego > 15) {
        wide_cam_requested = false;
      }
      wide_cam_requested = wide_cam_requested && sm["selfdriveState"].getSelfdriveState().getExperimentalMode() && frogpilot_toggles.value("camera_view").toInt() == 0;
    }
    // The Travel Assist menu's camera arm: 1 forces the wide road camera, 2 the narrow one, 0 leaves it to the above. Driver view
    // still wins
    int travel_assist_camera = frogpilot_nvg->travelAssistCameraMode;
    if (frogpilot_toggles.value("camera_view").toInt() == 1) {
      CameraWidget::setStreamType(VISION_STREAM_DRIVER);
    } else if (travel_assist_camera == 1 && has_wide_cam) {
      CameraWidget::setStreamType(VISION_STREAM_WIDE_ROAD);
    } else if (travel_assist_camera == 2) {
      CameraWidget::setStreamType(VISION_STREAM_ROAD);
    } else {
      CameraWidget::setStreamType(frogpilot_toggles.value("camera_view").toInt() == 3 || wide_cam_requested ? VISION_STREAM_WIDE_ROAD :
                                  VISION_STREAM_ROAD);
    }
    CameraWidget::setFrameId(sm["modelV2"].getModelV2().getFrameId());

    watchdog_stage("paint:before_begin_native");
    painter.beginNativePainting();
    watchdog_stage("paint:before_camera_paintgl");
    CameraWidget::paintGL();
    watchdog_stage("paint:after_camera_paintgl");
    painter.endNativePainting();
    watchdog_stage("paint:after_end_native");
  }

  watchdog_stage("paint:before_qpainter_draws");
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);

  // FrogPilot variables
  dmon.frogpilot_nvg = frogpilot_nvg;
  hud.frogpilot_nvg = frogpilot_nvg;
  model.frogpilot_nvg = frogpilot_nvg;

  experimental_btn->frogpilot_scene = frogpilot_scene;
  model.frogpilot_scene = frogpilot_scene;

  dmon.frogpilot_toggles = frogpilot_toggles;
  experimental_btn->frogpilot_toggles = frogpilot_toggles;
  hud.frogpilot_toggles = frogpilot_toggles;
  model.frogpilot_toggles = frogpilot_toggles;

  watchdog_stage("paint:before_model_draw");
  model.draw(painter, rect());
  watchdog_stage("paint:after_model_draw");
  dmon.draw(painter, rect());
  watchdog_stage("paint:after_dmon_draw");
  hud.updateState(*s);
  watchdog_stage("paint:after_hud_update_state");
  hud.draw(painter, rect());
  watchdog_stage("paint:after_hud_draw");

  // FrogPilot variables
  watchdog_stage("paint:before_frogpilot_widgets");
  frogpilot_nvg->paintFrogPilotWidgets(painter, *s);
  watchdog_stage("paint:after_frogpilot_widgets");

  // Debug: Draw DM distraction time
  auto dm_state = sm["driverMonitoringState"].getDriverMonitoringState();
  float distraction_time = dm_state.getDistractionTime();
  if (sm.alive("driverMonitoringState") && distraction_time > 0.1) {  // Only show if driver has looked away
    painter.save();
    painter.setPen(QColor(255, 255, 255, 200));
    painter.setFont(InterFont(40, QFont::Bold));
    QString debug_text = QString("DM: %1s").arg(QString::number(distraction_time, 'f', 1));
    QRect debug_rect(width() - 250, 50, 200, 50);
    painter.drawText(debug_rect, Qt::AlignRight | Qt::AlignTop, debug_text);
    painter.restore();
  }

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;

  // publish debug msg
  MessageBuilder msg;
  auto m = msg.initEvent().initUiDebug();
  m.setDrawTimeMillis(cur_draw_t - start_draw_t);
  watchdog_stage("paint:before_ui_debug_send");
  pm->send("uiDebug", msg);
  watchdog_stage("paint:end");
}

void AnnotatedCameraWidget::showEvent(QShowEvent *event) {
  CameraWidget::showEvent(event);

  ui_update_params(uiState());
  prev_draw_t = millis_since_boot();
}
