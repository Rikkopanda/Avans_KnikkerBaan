#include "mainWindow.h"

#include <QApplication>
#include <QString>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  QString portName;
  int baudRate = 115200;

  for (int i = 1; i < argc; ++i) {
    const QString arg = QString::fromLocal8Bit(argv[i]);
    if (arg == "--baud" && i + 1 < argc) {
      baudRate = QString::fromLocal8Bit(argv[++i]).toInt();
    } else if (!arg.startsWith('-') && portName.isEmpty()) {
      portName = arg;
    }
  }

  MainWindow w(portName, baudRate);
  w.show();
  return a.exec();
}
