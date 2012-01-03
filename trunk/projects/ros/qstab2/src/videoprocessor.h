#ifndef VIDEOPROCESSOR_H
#define VIDEOPROCESSOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class FrameProcessor {
public:
    // processing method
    virtual void process(cv:: Mat &input, cv:: Mat &output, bool isShow)= 0;
};


class VideoProcessor
{
private:
    cv::VideoCapture capture;

    std::vector<std::string> images;
    std::vector<std::string>::const_iterator itImg;

    float defoaltRate;
    void (*process)(cv::Mat&, cv::Mat&);
    bool callIt;
    std::string windowNameInput;
    std::string windowNameOutput;
    int delay;
    long fnumber;
    long frameToStop;
    bool stop;
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
    void setFrameProcessor(void (*frameProcessingCallback) (cv::Mat&, cv::Mat&));
    bool setInput(std::string filename);
    bool setInput(int id);
    bool setInput(const std::vector<std::string>& imgs);
    void setFrameProcessor(FrameProcessor* frameProcessorPtr);
    bool setOutput(const std::string &filename,  int codec=0,
                   double framerate=0.0, bool isColor=true);
    bool setOutput(const std::string &filename, const std::string &ext,
                    int numberOfDigits=3, int startIndex=0);
    void setDelay(int d);

    // Getters
    long getFrameNumber();
    float getFrameRate();
    cv::Size getFrameSize();
    int getCodec(char codec[4]);
    bool isStopped();
    bool isOpened();


    void writeNextFrame(cv::Mat& frame);
    void displayInput(std::string wn);
    void displayOutput(std::string wn);
    void dontDisplay();
    void run();
    bool runOnce();
    void stopIt();
    bool readNextFrame(cv::Mat& frame);
    void callProcess();
    void dontCallProcess();
    void stopAtFrameNo(long frame);

};

#endif // VIDEOPROCESSOR_H
