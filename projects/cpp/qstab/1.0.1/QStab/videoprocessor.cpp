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
    captures.clear();
    cv::VideoCapture capture;
    captures.push_back(capture);

    return captures[0].open(filename);
}

bool VideoProcessor::addInput(std::string filename) {
    fnumber = 0;
    cv::VideoCapture capture;
    captures.push_back(capture);

    return captures[captures.size()-1].open(filename);
}

bool VideoProcessor::setInput(int id)
{
    fnumber = 0;
    captures.clear();
    cv::VideoCapture capture;
    captures.push_back(capture);
    isCam = true;

    return captures[0].open(id);
}

bool VideoProcessor::addInput(int id) {
    fnumber = 0;
    cv::VideoCapture capture;
    captures.push_back(capture);
    isCam=true;

    return captures[captures.size()-1].open(id);
}

void VideoProcessor::displayInput(std::string wn)
{
    windowNameInputs.push_back(wn);
    cv::namedWindow(wm);
}

void VideoProcessor::displayOutput(std::string wn)
{
    windowNameOutputs.push_back(wn);
    cv::namedWindow(wm);
}

void VideoProcessor::dontDisplay()
{
    for(std::vector<std::string>::const_iterator it = windowNameInputs.begin();
                            it != windowNameInputs.end(); it++) {
        cv::destroyWindow(it);
    }
    for(std::vector<std::string>::const_iterator it = windowNameOutputs.begin();
                            it != windowNameOutputs.end(); it++) {
        cv::destroyWindow(it);
    }

    windowNameInputs.clear();
    windowNameOutputs.clear();
}

void VideoProcessor::run()
{
    std::vector<cv::Mat> frames;
    std::vector<cv::Mat> outputs;
    if (!isOpened())
        return;

    stop= false;

    while (!isStopped()) {

        if (!readNextFrames(frames))
            break;

        if (windowNameInputs.size()>0) {
            for(int i = 0; i < windowNameInputs.size(); i++){
                if(i<frames.size()) {
                    cv::imshow(windowNameInputs[i],frames[i]);
                }
            }
        }


        if (callIt) {
            if (process)
                process(frames, outputs);
            else if (frameProcessor)
                frameProcessor->process(frames,outputs, (windowNameOutputs.size()>0));
            fnumber++;
        } else {
            outputs = frames;
        }

        if (outputFile.length()!=0)
            writeNextFrame(outputs[0]);

        if (windowNameOutputs.size()>0)
            for(int i = 0; i < windowNameOutputs.size(); i++){
                if(i<outputs.size()) {
                    cv::imshow(windowNameOutputs[i],outputs[i]);
                }
            }

        if (delay>=0 && cv::waitKey(delay)>=0)
            stopIt();
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
    return !captures.empty() || !images.empty();
}

void VideoProcessor::setDelay(int d)
{
    delay = d;
}

bool VideoProcessor::readNextFrames(std::vector<cv::Mat> &frames)
{
    if (images.size()==0) {
        // Если хоть одна камера вышла из строя возвращается false
        bool ok = true;
        frames.clear();
        for(std::vector<cv::VideoCapture>::const_iterator it = captures.begin();
                        it != captures.end(); it++) {
            cv::Mat frame;
            ok = (it.read(frame)&&ok);
            frames.push_back(frame);
        }
        return ok;
    } else {
        if (itImg != images.end()) {
            frames.clear();
            frames.push_back(cv::imread(*itImg));
            itImg++;
            return frames[0].data != 0;
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

float VideoProcessor::getFrameRate()
{
    if(isCam) {
        return defoaltRate;
    } else {
        float rate = captures[0].get(CV_CAP_PROP_FPS);
        return rate?rate:defoaltRate;
    }
}

cv::Size VideoProcessor::getFrameSize() {
    cv::Mat frame;
    captures[0] >> frame;
    return frame.size();
}

bool VideoProcessor::setInput(const std::vector<std::string>& imgs) {
    fnumber = 0;
    captures.clear();
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


// Подумать над реализацией одновременной записи нескольких видео
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
                captures[0].get(CV_CAP_PROP_FOURCC));
    codec[0]= returned.code[0];
    codec[1]= returned.code[1];
    codec[2]= returned.code[2];
    codec[3]= returned.code[3];
    return returned.value;
}


