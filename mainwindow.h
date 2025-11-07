#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QAudioSink>
#include <QFile>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QComboBox>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QMutex>
#include <QElapsedTimer>


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    // === 串口 ===
    QSerialPort serial;

    // === 音频播放 ===
    QAudioSink *audioSink = nullptr;
    QIODevice *audioDevice = nullptr;

    // === WAV 文件保存 ===
    QFile wavFile;
    bool saveWav = false;
    QByteArray wavWriteBuffer;
    QMutex wavMutex;

    // === 波形显示 ===
    QChart *chart = nullptr;
    QChartView *chartView = nullptr;
    QLineSeries *series = nullptr;
    QValueAxis *axisX = nullptr;
    QValueAxis *axisY = nullptr;

    QElapsedTimer lastDataTimer;

    // === 原始字节环形缓冲区（核心）===
    static const int RAW_BUFFER_SIZE = 2000000;
    QVector<char> rawBuffer;
    int rawWriteIndex = 0;
    int rawReadIndex = 0;
    bool bufferWrapped = false;

    // === 配置参数 ===
    int maxPoints = 30000;
    int sampleRate = 8000;
    int sampleBits = 16;
    int channels = 1;

    // === GUI 控件 ===
    QComboBox *portCombo = nullptr;
    QComboBox *baudCombo = nullptr;
    QPushButton *openButton = nullptr;
    QPushButton *wavButton = nullptr;
    QSpinBox *pointsSpin = nullptr;

    // PCM 控制控件
    QComboBox *bitsCombo = nullptr;
    QSpinBox *sampleRateSpin = nullptr;
    QComboBox *endianCombo = nullptr;
    QSpinBox *offsetSpin = nullptr;

    // plot buffer
    static const int PLOT_BUFFER_SIZE = 10000000;
    QVector<qint16> plotBuffer;
    int plotWriteIndex = 0;
    bool plotBufferWrapped = false;

private slots:
    void readSerialData();
    void refreshPlot();
    void handleOpenSerial();
    void handleToggleWav();
    void handlePointsChanged(int points);

private:
    void setupGui();
    void scanSerialPorts();
    void initAudio();
    void updateWavHeader();
    void feedAudioDevice();
    void flushWavBuffer();
    void setMaxPoints(int points);

    QVector<qint16> extractSamples(int maxCount, bool consume = false);

    int readableBytes() const;
};

#endif // MAINWINDOW_H
