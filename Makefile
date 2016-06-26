CC=g++
CFLAGS=`pkg-config --cflags --libs opencv` -Wall -Wno-deprecated -lboost_thread -lboost_system -lpthread 
LDFLAGS= -L/usr/lib -L/usr/local/lib -I include/ -I/usr/include/vtk-5.8 -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-6.1
PCLFLAGS=`pkg-config --cflags --libs pcl_visualization-1.7 pcl_common-1.7 pcl_io-1.7`

VPATH = src:src/
vpath %.cpp	src/
vpath %.cpp	test/
vpath %.hpp	include/
vpath %.o	build/

OBJECTS = Camera.o Screen.o
SOURCES = $(OBJECTS: .o=.cpp)

$(OBJECTS):	$(SOURCES)
			$(CC) -c $^ $(CFLAGS) $(LDFLAGS)  -o $@
			mv $@ build/
			
testScreen:	Screen.cpp testScreen.cpp
			$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@
			mv $@ build/
			./build/$@
testTwin:	Screen.cpp Camera.cpp TwinCamera.cpp testTwin.cpp
			$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@
			mv $@ build/
			./build/$@

testCamera:	Camera.cpp Screen.cpp testCamera.cpp
			$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@
			mv $@ build/
			./build/$@
			
testStereo:	StereoDepth.cpp Preprocess.cpp Camera.cpp Screen.cpp testStereo.cpp
			$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@
			mv $@ build/	
					
testPclView:pclView.cpp testPclView.cpp
			$(CC) $^ -o $@ $(CFLAGS) $(LDFLAGS) $(PCLFLAGS)
			mv $@ build/

Stereomain:	StereoDepth.cpp TwinCamera.cpp Preprocess.cpp Camera.cpp Screen.cpp StereoMain.cpp
			$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@
			mv $@ build/

camera_calib:	TwinCamera.cpp Camera.cpp stereocalibrate.cpp
			$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@
			mv $@ build/

stereoParam:	StereoDepth.cpp Preprocess.cpp  Screen.cpp StereoParamSet.cpp
			$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@
			mv $@ build/

setEmptyTruck:	StereoDepth.cpp Preprocess.cpp Camera.cpp TwinCamera.cpp SetEmptyTruck.cpp
			$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@
			mv $@ build/

getPercetage:	Screen.cpp StereoDepth.cpp Preprocess.cpp Camera.cpp TwinCamera.cpp GetPercentage.cpp
			$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@
			mv $@ build/
