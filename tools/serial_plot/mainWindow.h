#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QByteArray>
#include <QElapsedTimer>
#include <QMainWindow>
#include <QMap>
#include <QString>
#include <QVector>
#include <QWidget>

class QLabel;
class QPaintEvent;
class QSlider;
class QTimer;

struct GraphSample
{
  double time_s;
  double setpoint;
  double actual;
};

class GraphWidget : public QWidget
{
public:
  explicit GraphWidget(QWidget *parent = nullptr);

  void addSample(double time_s, double setpoint, double actual);
  void setCurrentValues(double setpoint, double actual);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  QVector<GraphSample> samples_;
  double currentSetpoint_ = 16.0;
  double currentActual_ = 0.0;
  int maxSamples_ = 400;
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(const QString &portName = QString(), int baudRate = 115200, QWidget *parent = nullptr);
  ~MainWindow() override;

private slots:
  void sliderChanged();
  void readSerial();

private:
  QSlider *createSlider(const QString &name, double min, double max, double step, double value, QLabel **valueLabel);
  bool openSerial(const QString &portName, int baudRate);
  bool configureSerial(int baudRate);
  void closeSerial();
  void sendCommand(const QString &command);
  void processLine(const QString &line);
  bool parsePlotLine(const QString &line, QMap<QString, double> &fields) const;
  void updateSliderLabels();
  double sliderValue(const QSlider *slider, double step) const;

  GraphWidget *graph_ = nullptr;
  QSlider *setpointSlider_ = nullptr;
  QSlider *kpSlider_ = nullptr;
  QSlider *kiSlider_ = nullptr;
  QSlider *kdSlider_ = nullptr;
  QLabel *setpointValue_ = nullptr;
  QLabel *kpValue_ = nullptr;
  QLabel *kiValue_ = nullptr;
  QLabel *kdValue_ = nullptr;
  QTimer *serialTimer_ = nullptr;
  QElapsedTimer clock_;

  int serialFd_ = -1;
  QByteArray serialBuffer_;
  double actualPosition_ = 0.0;
  bool updatingFromSerial_ = false;
};
#endif // MAINWINDOW_H
