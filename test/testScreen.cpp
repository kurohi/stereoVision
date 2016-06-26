#include<Screen.hpp>

int main(int argc, char **argv){
	Screen scr;
	char c = 0;
	while(c != 27){
		c = waitKey(10);
	}
}
