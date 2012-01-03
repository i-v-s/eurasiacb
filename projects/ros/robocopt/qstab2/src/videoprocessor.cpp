#include "videoprocessor.h"
#include <iostream>
#include <iomanip>

VideoProcessor::VideoProcessor(): callIt(true), delay(0),
    fnumber(0), stop(false), frameToStop(-1), defoaltRate(15.), isCam(false)
{
}

void VideoProcessor::setFrameProcessor(void (*frameProcessingCallback)(cv::Mat &, cv::Mat &))
{
    frameProcessor= 0;
    process= frameProcessingCallback;
    callProcess();
}

bool VideoProcessor::setInput(std::string filename)
{
    fnumber = 0;
    capture.release();

    return capture.open(filename);
}

bool VideoProcessor::setInput(int id)
{
    fnumber = 0;
    capture.release();
    isCam = true;

    return capture.open(id);
}

void VideoProcessor::displayInput(std::string wn)
{
    windowNameInput = wn;
    cv::namedWindow(windowNameInput);
}

void VideoProcessor::displayOutput(std::string wn)
{
    windowNameOutput= wn;
    cv::namedWindow(windowNameOutput);
}

void VideoProcessor::dontDisplay()
{
    cv::destroyWindow(windowNameInput);
    cv::destroyWindow(windowNameOutput);
    windowNameInput.clear();
    windowNameOutput.clear();
}

bool VideoProcessor::runOnce() {
    cv::Mat frame;
    cv::Mat output;

    stop= false;

    if (!readNextFrame(frame))
        return false;

    if (windowNameInput.length()!=0)
        cv::imshow(windowNameInput,frame);

    if (callIt) {
        if (process)
            process(frame, output);
        else if (frameProcessor)
            frameProcessor->process(frame,output, (windowNameOutput.length()!=0));
        fnumber++;
    } else {
        output= frame;
    }

    if (outputFile.length()!=0) {
        writeNextFrame(output);
        cv::waitKey(10);
    }

    if (windowNameOutput.length()!=0)
        cv::imshow(windowNameOutput,output);

    if (delay>=0 && cv::waitKey(delay)>=0)
        stopIt();

    if (frameToStop>=0 && getFrameNumber()==frameToStop)
        stopIt();

    return true;
}

void VideoProcessor::run()
{
    cv::Mat frame;
    cv::Mat output;
    if (!isOpened())
        return;

    stop= false;

    while (!isStopped()) {
        if (!readNextFrame(frame))
            break;

        if (windowNameInput.length()!=0)
            cv::imshow(windowNameInput,frame);

        if (callIt) {
            if (process)
                process(frame, output);
            else if (frameProcessor)
                frameProcessor->process(frame,output, (windowNameOutput.length()!=0));
            fnumber++;
        } else {
            output= frame;
        }

        if (outputFile.length()!=0) {
            writeNextFrame(output);
        }

        if (windowNameOutput.length()!=0)
            cv::imshow(windowNameOutput,output);

        if (delay>=0 && cv::waitKey(delay)>=0)
            stopIt();

        if (frameToStop>=0 && getFrameNumber()==frameToStop)
            stopIt();

        cv::waitKey(10);
    }
}

void VideoProcessor::stopIt()
{
    stop = true;
}

bool VideoProcessor::isStopped()
{
    return stop;
}

bool VideoProcessor::isOpened()
{
    return capture.isOpened() || !images.empty();
}

void VideoProcessor::setDelay(int d)
{
    delay = d;
}

bool VideoProcessor::readNextFrame(cv::Mat &frame)
{
    if (images.size()==0)
        return capture.read(frame);
    else {
        if (itImg != images.end()) {
            frame= cv::imread(*itImg);
            itImg++;
            return frame.data != 0;
        } else {
            return false;
        }
    }
}

void VideoProcessor::callProcess()
{
    callIt = true;
}

void VideoProcessor::dontCallProcess()
{
    callIt = false;
}

void VideoProcessor::stopAtFrameNo(long frame)
{
    frameToStop = frame;
}

long VideoProcessor::getFrameNumber()
{
    long fnumber= static_cast<long>(capture.get(CV_CAP_PROP_POS_FRAMES));
    return fnumber;
}

float VideoProcessor::getFrameRate()
{
    if(isCam) {
        return defoaltRate;
    } else {
        float rate = capture.get(CV_CAP_PROP_FPS);
        return rate?rate:defoaltRate;
    }
}

cv::Size VideoProcessor::getFrameSize() {
    cv::Mat frame;
    capture >> frame;
    return frame.size();
}

bool VideoProcessor::setInput(const std::vector<std::string>& imgs) {
    fnumber = 0;
    capture.release();
    images= imgs;
    itImg= images.begin();
    return true;
}

void VideoProcessor::setFrameProcessor(FrameProcessor* frameProcessorPtr)
{
    process= 0;
    frameProcessor= frameProcessorPtr;
    callProcess();
}

bool VideoProcessor::setOutput(const std::string &filename,
            int codec, double framerate,
            bool isColor) {

    outputFile= filename;
    extension.clear();
    if (framerate==0.0)
        framerate= getFrameRate();
    char c[4];

    if (codec==0) {
        codec= getCodec(c);
    }

    return writer.open(outputFile,
                codec,
                framerate,
                getFrameSize(),
                isColor);
}

void VideoProcessor::writeNextFrame(cv::Mat& frame){
    if (extension.length()) {
        std::stringstream ss;
        ss << outputFile << std::setfill('0')
                    << std::setw(digits)
                    << currentIndex++ << extension;
        cv::imwrite(ss.str(),frame);
    } else {
        writer.write(frame);
    }
}

bool VideoProcessor::setOutput(const std::string &filename, const std::string &ext,
               int numberOfDigits, int startIndex) {
    if (numberOfDigits<0)
        return false;
    outputFile= filename;
    extension= ext;
    digits= numberOfDigits;
    currentIndex= startIndex;
    return true;
}

int VideoProcessor::getCodec(char codec[4]) {
    if (images.size()!=0) return -1;
    union {
            int value;
            char code[4];
        } returned;
    returned.value= static_cast<int>(
                capture.get(CV_CAP_PROP_FOURCC));
    codec[0]= returned.code[0];
    codec[1]= returned.code[1];
    codec[2]= returned.code[2];
    codec[3]= returned.code[3];
    return returned.value;
}


