#ifndef VIDEOPROCESSOR_H
#define VIDEOPROCESSOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class FrameProcessor {
public:
    // processing method
    virtual void process(std::vector<cv::Mat> &input, std::vector<cv::Mat> &output, bool isShow)= 0;
};


class VideoProcessor
{
private:
    std::vector<cv::VideoCapture> captures;

    std::vector<std::string> images;
    std::vector<std::string>::const_iterator itImg;

    float defoaltRate;
    void (*process)(cv::Mat&, cv::Mat&);
    bool callIt;
    std::vector<std::string> windowNameInputs;
    std::vector<std::string> windowNameOutputs;
    int delay;
    long fnumber;
    long frameToStop;
    bool stop;
    // Заплатка
    bool isCam;

    FrameProcessor* frameProcessor;

    // Write video
    cv::VideoWriter writer;
    std::string outputFile;
    int currentIndex;
    int digits;
    std::string extension;



public:
    VideoProcessor();

    //Setters
    void setFrameProcessor(void (*frameProcessingCallback) (std::vector<cv::Mat>&, std::vector<cv::Mat>&));
    bool setInput(std::string filename);
    bool setInput(int id);
    bool addInput(std::string filename);
    bool addInput(int id);
    bool setInput(const std::vector<std::string>& imgs);
    void setFrameProcessor(FrameProcessor* frameProcessorPtr);
    bool setOutput(const std::string &filename,  int codec=0,
                   double framerate=0.0, bool isColor=true);
    bool setOutput(const std::string &filename, const std::string &ext,
                    int numberOfDigits=3, int startIndex=0);
    void setDelay(int d);

    // Getters
    float getFrameRate();
    cv::Size getFrameSize();
    int getCodec(char codec[4]);
    bool isStopped();
    bool isOpened();


    void writeNextFrame(cv::Mat& frames);
    void displayInput(std::string wn);
    void displayOutput(std::string wn);
    void dontDisplay();
    void run();
    void stopIt();
    bool readNextFrames(std::vector<cv::Mat>& frames);
    void callProcess();
    void dontCallProcess();

};

#endif // VIDEOPROCESSOR_H
