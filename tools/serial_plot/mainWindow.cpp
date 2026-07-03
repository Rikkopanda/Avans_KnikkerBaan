#include "mainWindow.h"

#include <QApplication>
#include <QByteArray>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMap>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPushButton>
#include <QSlider>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

speed_t baudToTermios(int baudRate)
{
  switch (baudRate) {
  case 9600: return B9600;
  case 19200: return B19200;
  case 38400: return B38400;
  case 57600: return B57600;
  case 115200: return B115200;
#ifdef B230400
  case 230400: return B230400;
#endif
#ifdef B460800
  case 460800: return B460800;
#endif
#ifdef B921600
  case 921600: return B921600;
#endif
  default: return 0;
  }
}

double fieldOr(const QMap<QString, double> &fields, const QString &key, double fallback)
{
  return fields.contains(key) ? fields.value(key) : fallback;
}

QString formatNumber(double value, int decimals)
{
  return QString::number(value, 'f', decimals);
}

} // namespace

GraphWidget::GraphWidget(QWidget *parent)
  : QWidget(parent)
{
  setMinimumSize(640, 360);
  setAutoFillBackground(false);
}

void GraphWidget::addSample(double time_s, double setpoint, double actual)
{
  currentSetpoint_ = setpoint;
  currentActual_ = actual;
  samples_.append(GraphSample{time_s, setpoint, actual});
  while (samples_.size() > maxSamples_) {
    samples_.removeFirst();
  }
  update();
}

void GraphWidget::setCurrentValues(double setpoint, double actual)
{
  currentSetpoint_ = setpoint;
  currentActual_ = actual;
  update();
}

void GraphWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), QColor(18, 22, 26));

  const QRect plot = rect().adjusted(54, 24, -20, -46);
  painter.fillRect(plot, QColor(24, 31, 38));
  painter.setPen(QPen(QColor(67, 78, 89), 1));
  painter.drawRect(plot);

  for (int i = 1; i < 5; ++i) {
    const int y = plot.top() + (plot.height() * i) / 5;
    painter.drawLine(plot.left(), y, plot.right(), y);
  }
  for (int i = 1; i < 6; ++i) {
    const int x = plot.left() + (plot.width() * i) / 6;
    painter.drawLine(x, plot.top(), x, plot.bottom());
  }

  double minTime = 0.0;
  double maxTime = 20.0;
  double minValue = 0.0;
  double maxValue = 30.0;

  if (!samples_.isEmpty()) {
    maxTime = samples_.last().time_s;
    minTime = qMax(0.0, maxTime - 20.0);
    minValue = samples_.first().actual;
    maxValue = samples_.first().actual;
    for (const GraphSample &sample : samples_) {
      minValue = qMin(minValue, qMin(sample.setpoint, sample.actual));
      maxValue = qMax(maxValue, qMax(sample.setpoint, sample.actual));
    }
    minValue = qMin(minValue, currentSetpoint_);
    maxValue = qMax(maxValue, currentSetpoint_);
    const double padding = qMax(1.0, (maxValue - minValue) * 0.15);
    minValue -= padding;
    maxValue += padding;
  }

  if (qFuzzyCompare(minValue, maxValue)) {
    minValue -= 1.0;
    maxValue += 1.0;
  }

  auto toPoint = [&](double time_s, double value) {
    const double xRatio = (time_s - minTime) / qMax(0.001, maxTime - minTime);
    const double yRatio = (value - minValue) / qMax(0.001, maxValue - minValue);
    return QPointF(plot.left() + xRatio * plot.width(), plot.bottom() - yRatio * plot.height());
  };

  auto drawLine = [&](bool setpointLine, const QColor &color) {
    QPainterPath path;
    bool started = false;
    for (const GraphSample &sample : samples_) {
      if (sample.time_s < minTime) {
        continue;
      }
      const QPointF point = toPoint(sample.time_s, setpointLine ? sample.setpoint : sample.actual);
      if (!started) {
        path.moveTo(point);
        started = true;
      } else {
        path.lineTo(point);
      }
    }
    painter.setPen(QPen(color, 2.4));
    painter.drawPath(path);
  };

  drawLine(true, QColor(246, 174, 45));
  drawLine(false, QColor(76, 201, 240));

  painter.setPen(QColor(232, 237, 242));
  painter.drawText(12, 18, "Position graph");
  painter.setPen(QColor(246, 174, 45));
  painter.drawText(plot.left(), height() - 18, "Setpoint");
  painter.setPen(QColor(76, 201, 240));
  painter.drawText(plot.left() + 90, height() - 18, "Actual");

  painter.setPen(QColor(154, 167, 178));
  painter.drawText(8, plot.top() + 8, formatNumber(maxValue, 1));
  painter.drawText(8, plot.bottom(), formatNumber(minValue, 1));
  painter.drawText(plot.right() - 150, height() - 18,
                   QString("Set %1 cm  Actual %2 cm")
                     .arg(formatNumber(currentSetpoint_, 2))
                     .arg(formatNumber(currentActual_, 2)));
}

MainWindow::MainWindow(const QString &portName, int baudRate, QWidget *parent)
  : QMainWindow(parent)
{
  setWindowTitle("Serial Plot");
  resize(1000, 620);

  QWidget *central = new QWidget(this);
  QVBoxLayout *root = new QVBoxLayout(central);

  graph_ = new GraphWidget(central);
  root->addWidget(graph_, 1);

  QGroupBox *controls = new QGroupBox("Control", central);
  QGridLayout *grid = new QGridLayout(controls);

  setpointSlider_ = createSlider("Setpoint", 0.0, 30.0, 0.1, 16.0, &setpointValue_);
  kpSlider_ = createSlider("Kp", 0.0, 20.0, 0.05, 3.0, &kpValue_);
  kiSlider_ = createSlider("Ki", 0.0, 10.0, 0.01, 0.15, &kiValue_);
  kdSlider_ = createSlider("Kd", 0.0, 10.0, 0.05, 3.5, &kdValue_);

  grid->addWidget(new QLabel("Setpoint"), 0, 0);
  grid->addWidget(setpointSlider_, 0, 1);
  grid->addWidget(setpointValue_, 0, 2);
  grid->addWidget(new QLabel("Kp"), 1, 0);
  grid->addWidget(kpSlider_, 1, 1);
  grid->addWidget(kpValue_, 1, 2);
  grid->addWidget(new QLabel("Ki"), 2, 0);
  grid->addWidget(kiSlider_, 2, 1);
  grid->addWidget(kiValue_, 2, 2);
  grid->addWidget(new QLabel("Kd"), 3, 0);
  grid->addWidget(kdSlider_, 3, 1);
  grid->addWidget(kdValue_, 3, 2);
  grid->setColumnStretch(1, 1);

  root->addWidget(controls);
  setCentralWidget(central);

  connect(setpointSlider_, SIGNAL(valueChanged(int)), this, SLOT(sliderChanged()));
  connect(kpSlider_, SIGNAL(valueChanged(int)), this, SLOT(sliderChanged()));
  connect(kiSlider_, SIGNAL(valueChanged(int)), this, SLOT(sliderChanged()));
  connect(kdSlider_, SIGNAL(valueChanged(int)), this, SLOT(sliderChanged()));

  updateSliderLabels();
  clock_.start();
  graph_->addSample(0.0, sliderValue(setpointSlider_, 0.1), actualPosition_);

  serialTimer_ = new QTimer(this);
  connect(serialTimer_, SIGNAL(timeout()), this, SLOT(readSerial()));
  serialTimer_->start(30);

  if (!portName.isEmpty() && openSerial(portName, baudRate)) {
    statusBar()->showMessage(QString("Connected to %1 at %2 baud").arg(portName).arg(baudRate));
  } else if (!portName.isEmpty()) {
    statusBar()->showMessage(QString("Failed to open %1").arg(portName));
  } else {
    statusBar()->showMessage("No serial port selected. Start with: ./serial_plot_gui /dev/ttyUSB0");
  }
}

MainWindow::~MainWindow()
{
  closeSerial();
}

QSlider *MainWindow::createSlider(const QString &, double min, double max, double step, double value, QLabel **valueLabel)
{
  QSlider *slider = new QSlider(Qt::Horizontal, this);
  slider->setRange(qRound(min / step), qRound(max / step));
  slider->setValue(qRound(value / step));
  slider->setSingleStep(1);
  slider->setPageStep(10);

  *valueLabel = new QLabel(this);
  (*valueLabel)->setMinimumWidth(72);
  (*valueLabel)->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  return slider;
}

double MainWindow::sliderValue(const QSlider *slider, double step) const
{
  return slider->value() * step;
}

void MainWindow::updateSliderLabels()
{
  setpointValue_->setText(formatNumber(sliderValue(setpointSlider_, 0.1), 1) + " cm");
  kpValue_->setText(formatNumber(sliderValue(kpSlider_, 0.05), 2));
  kiValue_->setText(formatNumber(sliderValue(kiSlider_, 0.01), 2));
  kdValue_->setText(formatNumber(sliderValue(kdSlider_, 0.05), 2));
}

void MainWindow::sliderChanged()
{
  updateSliderLabels();

  const double setpoint = sliderValue(setpointSlider_, 0.1);
  graph_->setCurrentValues(setpoint, actualPosition_);

  if (updatingFromSerial_) {
    return;
  }

  QObject *who = sender();
  if (who == setpointSlider_) {
    graph_->addSample(clock_.elapsed() / 1000.0, setpoint, actualPosition_);
    sendCommand(QString("set:%1").arg(formatNumber(setpoint, 2)));
  } else if (who == kpSlider_) {
    sendCommand(QString("kp:%1").arg(formatNumber(sliderValue(kpSlider_, 0.05), 2)));
  } else if (who == kiSlider_) {
    sendCommand(QString("ki:%1").arg(formatNumber(sliderValue(kiSlider_, 0.01), 3)));
  } else if (who == kdSlider_) {
    sendCommand(QString("kd:%1").arg(formatNumber(sliderValue(kdSlider_, 0.05), 2)));
  }
}

bool MainWindow::openSerial(const QString &portName, int baudRate)
{
  serialFd_ = ::open(portName.toLocal8Bit().constData(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serialFd_ < 0) {
    return false;
  }

  if (!configureSerial(baudRate)) {
    closeSerial();
    return false;
  }

  return true;
}

bool MainWindow::configureSerial(int baudRate)
{
  const speed_t speed = baudToTermios(baudRate);
  if (speed == 0) {
    return false;
  }

  termios tty {};
  if (tcgetattr(serialFd_, &tty) != 0) {
    return false;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  return tcsetattr(serialFd_, TCSANOW, &tty) == 0;
}

void MainWindow::closeSerial()
{
  if (serialFd_ >= 0) {
    ::close(serialFd_);
    serialFd_ = -1;
  }
}

void MainWindow::sendCommand(const QString &command)
{
  if (serialFd_ < 0) {
    return;
  }

  const QByteArray payload = command.toUtf8() + '\n';
  ::write(serialFd_, payload.constData(), payload.size());
}

void MainWindow::readSerial()
{
  if (serialFd_ < 0) {
    return;
  }

  char data[512];
  while (true) {
    const ssize_t count = ::read(serialFd_, data, sizeof(data));
    if (count > 0) {
      serialBuffer_.append(data, static_cast<int>(count));
    } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;
    } else {
      break;
    }
  }

  int newline = -1;
  while ((newline = serialBuffer_.indexOf('\n')) >= 0) {
    QByteArray rawLine = serialBuffer_.left(newline);
    serialBuffer_.remove(0, newline + 1);
    if (rawLine.endsWith('\r')) {
      rawLine.chop(1);
    }
    processLine(QString::fromUtf8(rawLine));
  }
}

bool MainWindow::parsePlotLine(const QString &line, QMap<QString, double> &fields) const
{
  fields.clear();
  const QString clean = line.trimmed();
  if (!clean.startsWith("Plot,")) {
    return false;
  }

  const QStringList tokens = clean.mid(5).split(',', Qt::SkipEmptyParts);
  for (const QString &token : tokens) {
    const int colon = token.indexOf(':');
    if (colon <= 0) {
      continue;
    }
    bool ok = false;
    const QString key = token.left(colon).trimmed();
    const double value = token.mid(colon + 1).trimmed().toDouble(&ok);
    if (ok) {
      fields.insert(key, value);
    }
  }

  return !fields.isEmpty();
}

void MainWindow::processLine(const QString &line)
{
  QMap<QString, double> fields;
  if (!parsePlotLine(line, fields)) {
    return;
  }

  const double setpoint = fieldOr(fields, "Set", sliderValue(setpointSlider_, 0.1));
  actualPosition_ = fieldOr(fields, "DistF", fieldOr(fields, "Dist", actualPosition_));

  updatingFromSerial_ = true;
  if (fields.contains("Set")) {
    setpointSlider_->setValue(qRound(setpoint / 0.1));
  }
  if (fields.contains("Kp")) {
    kpSlider_->setValue(qRound(fields.value("Kp") / 0.05));
  }
  if (fields.contains("Ki")) {
    kiSlider_->setValue(qRound(fields.value("Ki") / 0.01));
  }
  if (fields.contains("Kd")) {
    kdSlider_->setValue(qRound(fields.value("Kd") / 0.05));
  }
  updatingFromSerial_ = false;
  updateSliderLabels();

  graph_->addSample(clock_.elapsed() / 1000.0, sliderValue(setpointSlider_, 0.1), actualPosition_);
  statusBar()->showMessage(QString("Set %1 cm | Actual %2 cm | Err %3 | Servo %4")
                             .arg(formatNumber(sliderValue(setpointSlider_, 0.1), 2))
                             .arg(formatNumber(actualPosition_, 2))
                             .arg(formatNumber(fieldOr(fields, "Err", setpoint - actualPosition_), 2))
                             .arg(formatNumber(fieldOr(fields, "Servo", 0.0), 1)));
}
