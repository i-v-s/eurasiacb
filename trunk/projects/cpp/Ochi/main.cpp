#include <cstdio>
#include <iostream>
#include <png.h>

using namespace std;


int main() {

	FILE* file;
	file = fopen("../libviso2/img/I1_000000.png","rb");
	if (!file)
	{
	   cout << "Cannot find image" << endl;
	}

	unsigned char* header;
	int number;

	fread(header, 1, number, file);
	bool is_png = !png_sig_cmp(header, 0, number);

	if(!is_png) {
		   cout << "It's not a png-file" << endl;
	}

}
