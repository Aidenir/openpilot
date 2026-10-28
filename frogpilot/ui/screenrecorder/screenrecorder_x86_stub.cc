// Off the device only (selfdrive/ui/SConscript): the OpenMAX screen recorder can't be built on a PC, so it does nothing there
#pragma clang diagnostic ignored "-Wunused-private-field"
#include "frogpilot/ui/screenrecorder/screenrecorder.h"

ScreenRecorder::ScreenRecorder(QWidget *parent) : QPushButton(parent) {}
void ScreenRecorder::startRecording() {}
void ScreenRecorder::stopRecording() {}
void ScreenRecorder::paintEvent(QPaintEvent *event) {}
void ScreenRecorder::toggleRecording() {}
void ScreenRecorder::encodeImage() {}
void ScreenRecorder::updateState() {}
OmxEncoder::~OmxEncoder() {}

