#include <cstdio>
#include <iostream>

using namespace std;


int main() {
	FILE* file;
	file = fopen("../libviso2/img/I1_000000.png","rb");
	if (!file) cout << "i'm here";
	else cout << "i'm not here";
	return 1;
}
