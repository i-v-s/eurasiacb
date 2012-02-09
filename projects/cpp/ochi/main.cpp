#include "matcher.h"
#include "pngUtils.h"
#include <matchimages.h>

//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main()
{

   Rectify rec;
   MatchImages mi9;
   mi9.match_images("/home/vasiliy/svn/eurasiacb/projects/cpp/libviso2/img/I1_000000.png",
                    "/home/vasiliy/svn/eurasiacb/projects/cpp/libviso2/img/I1_000000.png",
                    1, rec, 10, 20.0, 20.0, 1);
   return 1;
}


