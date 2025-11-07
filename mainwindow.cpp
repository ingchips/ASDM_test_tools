#include "mainwindow.h"
#include <QTimer>
#include <QByteArray>
#include <QDataStream>
#include <QtEndian>
#include <QDebug>
#include <QSerialPortInfo>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <algorithm>
#include <QMutexLocker>

static const int RAW_BUFFER_SIZE = 2000000;      // 原始字节缓冲区（2MB）
static const int PLOT_BUFFER_SIZE = 200000;      // 波形样本缓冲区（200k 样本）
static const int WAV_BUFFER_THRESHOLD = 8192;    // WAV 写入阈值

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    audioSink(nullptr),
    audioDevice(nullptr),
    saveWav(false),
    maxPoints(30000),
    sampleRate(8000),
    sampleBits(16),
    channels(1)
{
    setWindowIcon(QIcon("ingchips.ico"));
    // 初始化原始字节缓冲区
    rawBuffer.resize(RAW_BUFFER_SIZE);
    // 初始化波形样本缓冲区
    plotBuffer.resize(PLOT_BUFFER_SIZE);
    // smoothBuffer.resize(SMOOTH_WINDOW);

    // 初始化图表
    series = new QLineSeries();
    chart = new QChart();
    chart->addSeries(series);
    axisX = new QValueAxis();
    axisY = new QValueAxis();
    chart->setAxisX(axisX, series);
    chart->setAxisY(axisY, series);
    axisX->setRange(0, maxPoints);
    axisY->setRange(-32768, 32767);
    chartView = new QChartView(chart);

    setupGui();
    setMinimumSize(200, 200);
    resize(800, 600);

    connect(&serial, &QSerialPort::readyRead, this, &MainWindow::readSerialData);

    QTimer *plotTimer = new QTimer(this);
    connect(plotTimer, &QTimer::timeout, this, &MainWindow::refreshPlot);
    plotTimer->start(30); // 33 FPS

    QTimer *audioTimer = new QTimer(this);
    connect(audioTimer, &QTimer::timeout, this, &MainWindow::feedAudioDevice);
    audioTimer->start(5); // 每5ms喂音频
}

MainWindow::~MainWindow()
{
    if (audioDevice) audioDevice->close();
    if (audioSink) delete audioSink;
    if (wavFile.isOpen()) {
        flushWavBuffer();
        updateWavHeader();
        wavFile.close();
    }
}

void MainWindow::setupGui()
{
    QWidget *central = new QWidget();
    QVBoxLayout *mainLayout = new QVBoxLayout(central);

    QHBoxLayout *ctrlLayout = new QHBoxLayout();

    portCombo = new QComboBox();
    scanSerialPorts();
    ctrlLayout->addWidget(new QLabel("串口:"));
    ctrlLayout->addWidget(portCombo);

    baudCombo = new QComboBox();
    baudCombo->addItems({"9600","19200","38400","57600","115200","230400","460800","921600",
                         "1152000","1000000","1500000","2000000","2500000","3000000"});
    baudCombo->setCurrentText("1500000");
    ctrlLayout->addWidget(new QLabel("波特率:"));
    ctrlLayout->addWidget(baudCombo);

    openButton = new QPushButton("打开串口");
    ctrlLayout->addWidget(openButton);
    connect(openButton, &QPushButton::clicked, this, &MainWindow::handleOpenSerial);

    wavButton = new QPushButton("开始保存 WAV");
    ctrlLayout->addWidget(wavButton);
    connect(wavButton, &QPushButton::clicked, this, &MainWindow::handleToggleWav);

    pointsSpin = new QSpinBox();
    pointsSpin->setRange(100, 100000);
    pointsSpin->setValue(maxPoints);
    ctrlLayout->addWidget(new QLabel("显示采样点数:"));
    ctrlLayout->addWidget(pointsSpin);
    connect(pointsSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::handlePointsChanged);

    mainLayout->addLayout(ctrlLayout);
    mainLayout->addWidget(chartView);

    QHBoxLayout *pcmLayout = new QHBoxLayout();
    bitsCombo = new QComboBox();
    bitsCombo->addItems({"8", "16"});
    bitsCombo->setCurrentText(QString::number(sampleBits));
    pcmLayout->addWidget(new QLabel("位宽:"));
    pcmLayout->addWidget(bitsCombo);
    connect(bitsCombo, &QComboBox::currentTextChanged, this, [this](const QString &text){
        sampleBits = text.toInt();
        initAudio();
    });

    sampleRateSpin = new QSpinBox();
    sampleRateSpin->setRange(2000, 192000);
    sampleRateSpin->setValue(sampleRate);
    pcmLayout->addWidget(new QLabel("采样率:"));
    pcmLayout->addWidget(sampleRateSpin);
    connect(sampleRateSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int rate){
        sampleRate = rate;
        initAudio();
    });

    endianCombo = new QComboBox();
    endianCombo->addItems({"小端", "大端"});
    pcmLayout->addWidget(new QLabel("字节序:"));
    pcmLayout->addWidget(endianCombo);

    offsetSpin = new QSpinBox();
    offsetSpin->setRange(0, 16);
    offsetSpin->setValue(0);
    pcmLayout->addWidget(new QLabel("字节偏移:"));
    pcmLayout->addWidget(offsetSpin);

    mainLayout->addLayout(pcmLayout);
    setCentralWidget(central);
}

void MainWindow::scanSerialPorts()
{
    portCombo->clear();
    for (const auto &info : QSerialPortInfo::availablePorts())
        portCombo->addItem(info.portName());
}

void MainWindow::handleOpenSerial()
{
    if (serial.isOpen()) {
        serial.close();
        openButton->setText("打开串口");
        return;
    }

    serial.setPortName(portCombo->currentText());
    serial.setBaudRate(baudCombo->currentText().toInt());
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    if (serial.open(QIODevice::ReadOnly)) {
        openButton->setText("关闭串口");
        initAudio();
    } else {
        qDebug() << "串口打开失败";
    }
}

void MainWindow::handleToggleWav()
{
    if (!saveWav) {
        wavFile.setFileName("record.wav");
        if (wavFile.open(QIODevice::WriteOnly)) {
            QByteArray header(44, 0);
            wavFile.write(header);
            saveWav = true;
            wavButton->setText("停止保存 WAV");
        }
    } else {
        saveWav = false;
        flushWavBuffer();
        updateWavHeader();
        wavFile.close();
        wavButton->setText("开始保存 WAV");
    }
}

void MainWindow::handlePointsChanged(int points)
{
    setMaxPoints(points);
}

void MainWindow::initAudio()
{
    QAudioFormat fmt;
    fmt.setSampleRate(sampleRate);
    fmt.setChannelCount(channels);
    fmt.setSampleFormat(sampleBits == 16 ? QAudioFormat::Int16 : QAudioFormat::UInt8);

    if (audioSink) delete audioSink;
    audioSink = new QAudioSink(fmt, this);
    audioDevice = audioSink->start();
}

// 计算原始缓冲区可读字节数
int MainWindow::readableBytes() const
{
    if (rawWriteIndex >= rawReadIndex)
        return rawWriteIndex - rawReadIndex;
    else
        return RAW_BUFFER_SIZE - rawReadIndex + rawWriteIndex;
}

// 从原始缓冲区提取并消费对齐样本
QVector<qint16> MainWindow::extractSamples(int maxCount, bool consume)
{
    int bytesPerSample = sampleBits / 8;
    int offset = offsetSpin->value();
    if (offset < 0) offset = 0;

    int avail = readableBytes();
    if (avail < offset + bytesPerSample) return {};

    int maxSamples = qMin(maxCount, (avail - offset) / bytesPerSample);
    if (maxSamples <= 0) return {};

    QVector<qint16> samples;
    samples.reserve(maxSamples);

    bool littleEndian = (endianCombo->currentText() == "小端");
    int readPos = (rawReadIndex + offset) % RAW_BUFFER_SIZE;

    for (int i = 0; i < maxSamples; ++i) {
        qint16 val = 0;
        if (sampleBits == 16) {
            uchar b0 = static_cast<uchar>(rawBuffer[readPos]);
            uchar b1 = static_cast<uchar>(rawBuffer[(readPos + 1) % RAW_BUFFER_SIZE]);
            val = littleEndian ? static_cast<qint16>(b0 | (b1 << 8))
                               : static_cast<qint16>((b0 << 8) | b1);
        } else if (sampleBits == 8) {
            val = static_cast<qint16>(static_cast<qint8>(rawBuffer[readPos]));
        }
        samples.append(val);
        readPos = (readPos + bytesPerSample) % RAW_BUFFER_SIZE;
    }

    if (consume) {
        rawReadIndex = (rawReadIndex + offset + maxSamples * bytesPerSample) % RAW_BUFFER_SIZE;
    }

    return samples;
}

// === 波形刷新：从 plotBuffer 读取历史数据 ===
void MainWindow::refreshPlot()
{
    int totalSamples = plotBufferWrapped ? PLOT_BUFFER_SIZE : plotWriteIndex;
    if (totalSamples == 0) {
        series->clear();
        return;
    }

    int displayCount = qMin(totalSamples, maxPoints);
    int startIndex = (plotWriteIndex - displayCount + PLOT_BUFFER_SIZE) % PLOT_BUFFER_SIZE;

    QVector<QPointF> points;
    points.reserve(displayCount);
    for (int i = 0; i < displayCount; ++i) {
        int idx = (startIndex + i) % PLOT_BUFFER_SIZE;
        points.append(QPointF(i, static_cast<qreal>(plotBuffer[idx])));
    }

    axisX->setRange(0, points.size());

    // Y轴防抖
    static qint16 lastMin = 0, lastMax = 0;
    static int stableFrameCount = 0;
    static const int STABLE_FRAMES = 5;

    if (!points.isEmpty()) {
        auto [minIt, maxIt] = std::minmax_element(points.begin(), points.end(),
                                                  [](const QPointF& a, const QPointF& b) { return a.y() < b.y(); });
        qint16 minVal = static_cast<qint16>(minIt->y());
        qint16 maxVal = static_cast<qint16>(maxIt->y());
        minVal -= 150; maxVal += 150;

        bool rangeChanged = (qAbs(minVal - lastMin) > 200 || qAbs(maxVal - lastMax) > 200);
        if (rangeChanged) stableFrameCount = 0; else stableFrameCount++;
        if (stableFrameCount >= STABLE_FRAMES || rangeChanged) {
            axisY->setRange(minVal, maxVal);
            lastMin = minVal; lastMax = maxVal;
        }
    }

    series->replace(points);
}

// === 音频播放：消费原始数据，并写入 plotBuffer ===
#define ENABLE_SMOOTH_FILTER 0

void MainWindow::readSerialData()
{
    QByteArray data = serial.readAll();
    if (data.isEmpty()) return;

    for (char b : data) {
        rawBuffer[rawWriteIndex] = b;
        rawWriteIndex = (rawWriteIndex + 1) % RAW_BUFFER_SIZE;
        if (rawWriteIndex == 0) bufferWrapped = true;
    }
    lastDataTimer.restart(); // 更新最后接收时间
}

// === 核心：音频播放 + 静音填充 + 平滑滤波 ===
void MainWindow::feedAudioDevice()
{
    if (!audioDevice || !audioDevice->isOpen()) return;

    int samplesNeeded = audioDevice->bytesToWrite() / static_cast<int>(sizeof(qint16));
    if (samplesNeeded <= 0) return;

    // 尝试从原始缓冲区提取真实样本
    QVector<qint16> realSamples = extractSamples(samplesNeeded, true);
    int realCount = realSamples.size();

    // 准备输出缓冲区：默认全静音（0）
    QVector<qint16> outputSamples(samplesNeeded, 0);

    if (realCount > 0) {
        // === 应用平滑滤波（如果启用）===
#if ENABLE_SMOOTH_FILTER
        static const int SMOOTH_WINDOW = 5;
        static QVector<qint16> smoothBuf(SMOOTH_WINDOW);
        static int smoothIdx = 0;
        static bool smoothFilled = false;

        // 对每个真实样本进行平滑
        for (int i = 0; i < realCount; ++i) {
            qint16 raw = realSamples[i];
            smoothBuf[smoothIdx] = raw;
            smoothIdx = (smoothIdx + 1) % SMOOTH_WINDOW;
            if (smoothIdx == 0) smoothFilled = true;

            qint16 smoothed;
            if (smoothFilled) {
                qint64 sum = 0;
                for (qint16 s : smoothBuf) sum += s;
                smoothed = static_cast<qint16>(sum / SMOOTH_WINDOW);
            } else {
                qint64 sum = 0;
                for (int j = 0; j < smoothIdx; ++j) sum += smoothBuf[j];
                smoothed = (smoothIdx > 0) ? static_cast<qint16>(sum / smoothIdx) : raw;
            }
            outputSamples[i] = smoothed;

            // 写入波形缓冲区（平滑后数据）
            plotBuffer[plotWriteIndex] = smoothed;
            plotWriteIndex = (plotWriteIndex + 1) % PLOT_BUFFER_SIZE;
            if (plotWriteIndex == 0) plotBufferWrapped = true;
        }
#else
        // 无平滑：直接复制
        std::copy(realSamples.begin(), realSamples.end(), outputSamples.begin());
        for (qint16 s : realSamples) {
            plotBuffer[plotWriteIndex] = s;
            plotWriteIndex = (plotWriteIndex + 1) % PLOT_BUFFER_SIZE;
            if (plotWriteIndex == 0) plotBufferWrapped = true;
        }
#endif

        // === 保存真实数据到 WAV（不含静音）===
        if (saveWav && wavFile.isOpen()) {
            QByteArray buf(reinterpret_cast<const char*>(outputSamples.data()),
                           realCount * sizeof(qint16));
            QMutexLocker locker(&wavMutex);
            wavWriteBuffer.append(buf);
            if (wavWriteBuffer.size() >= WAV_BUFFER_THRESHOLD) {
                wavFile.write(wavWriteBuffer);
                wavWriteBuffer.clear();
            }
        }
    }
    // 如果 realCount == 0，则 outputSamples 全为 0（静音），且不更新 plotBuffer/WAV

    // 播放（含静音填充）
    audioDevice->write(reinterpret_cast<const char*>(outputSamples.data()),
                       outputSamples.size() * sizeof(qint16));
}


void MainWindow::flushWavBuffer()
{
    QMutexLocker locker(&wavMutex);
    if (!wavWriteBuffer.isEmpty()) {
        wavFile.write(wavWriteBuffer);
        wavWriteBuffer.clear();
    }
}

void MainWindow::updateWavHeader()
{
    if (!wavFile.isOpen()) return;

    qint64 dataSize = wavFile.size() - 44;
    wavFile.seek(0);
    QDataStream out(&wavFile);
    out.setByteOrder(QDataStream::LittleEndian);

    out.writeRawData("RIFF", 4);
    out << quint32(dataSize + 36);
    out.writeRawData("WAVE", 4);
    out.writeRawData("fmt ", 4);
    out << quint32(16);
    out << quint16(1);
    out << quint16(channels);
    out << quint32(sampleRate);
    out << quint32(sampleRate * channels * (sampleBits / 8));
    out << quint16(channels * (sampleBits / 8));
    out << quint16(sampleBits);
    out.writeRawData("data", 4);
    out << quint32(dataSize);
}

void MainWindow::setMaxPoints(int points)
{
    maxPoints = points;
    refreshPlot(); // 不再设置 axisX，由 refreshPlot 控制
}
