extern "C" {
#include "remoteApi/extApi.h"
}
#include <myo/myo.hpp>
#include <stdio.h>
#include <iostream>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>

using namespace std;
//using namespace cv;

class DataCollector : public myo::DeviceListener {
public:
	DataCollector()
		: onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
	{
	}

	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* myo, uint64_t timestamp)
	{
		// We've lost a Myo.
		// Let's clean up some leftover state.
		roll_w = 0;
		pitch_w = 0;
		yaw_w = 0;
		onArm = false;
		isUnlocked = false;
	}

	// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
	// as a unit quaternion.

	// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
	// making a fist, or not making a fist anymore.
	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
	{
		currentPose = pose;

		if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {

			myo->unlock(myo::Myo::unlockHold);

			myo->notifyUserAction();
		}
		else {

			myo->unlock(myo::Myo::unlockHold);
		}
	}

	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
		myo::WarmupState warmupState)
	{
		onArm = true;
		whichArm = arm;
	}

	void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
	{
		onArm = false;
	}

	void onUnlock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = true;
	}

	void onLock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = false;
	}

	void print()
	{
		// Clear the current line
		std::cout << '\r';
		std::string poseString = currentPose.toString();

		std::cout << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
		std::cout << std::flush;
	}

	string getPose() {
		std::cout << '[' << currentPose.toString() << std::string(14 - currentPose.toString().size(), ' ') << ']';
		return currentPose.toString();
	}

	bool onArm;
	myo::Arm whichArm;

	bool isUnlocked;

	// These values are set by onOrientationData() and onPose() above.
	int roll_w, pitch_w, yaw_w;
	myo::Pose currentPose;
};

int main(int argc, char **argv)
{
		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.example.hello-myo");

		std::cout << "Attempting to find a Myo..." << std::endl;

		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub.waitForMyo(10000);
		myo->unlock(myo::Myo::unlockHold);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}

		// We've found a Myo.
		std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector;

		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);

		// Finally we enter our main loop.

		// If a standard exception occurred, we print out its message and exit.
	
	
	int armJoints[6];
	string serverIP = "127.0.0.1";
	int serverPort = 19997;
	float position;
	int ang[6] = { 0, 0, 0, 0, 0, 0 };
	int currentjoint = 0;
	char jointname[50];
	
	simxFinish(-1); //closes all connections
	int clientID = simxStart((simxChar*)serverIP.c_str(), serverPort, true, true, 2000, 5);
	//Mat blah(200, 200, CV_8UC3);
	//imshow("blah", blah);
	//waitKey(0);


	for (int i = 0; i < 6; i++) {
		sprintf(jointname, "IRB140_joint%d", i + 1);
		simxGetObjectHandle(clientID, jointname, &armJoints[i], simx_opmode_blocking);
	}

	while (clientID != -1) {

		//simxAddStatusbarMessage(clientID, "Hello, V-rep can you hear me", simx_opmode_oneshot); 


		//simxGetObjectPosition(clientId, objectHandle, &position, simx_opmode_buffer);
		hub.run(1000 / 2);
		// After processing events, we call the print() member function we defined above to print out the values we've
		// obtained from any events that have occurred.
		string pose = collector.getPose();

		//int c = waitKey(0);
		std::cout << pose << std::endl;
		if (pose == "waveIn") // o
			ang[currentjoint] += 1;
		else if (pose == "waveOut")  // i
			ang[currentjoint] -= 1;
		else if (pose == "fingersSpread") {
			currentjoint = (currentjoint + 1) % 6;
			cout << currentjoint << endl;
		}
		simxSetJointPosition(clientID, armJoints[currentjoint], ang[currentjoint]*3.14 / 180, simx_opmode_oneshot);
		simxGetJointPosition(clientID, armJoints[currentjoint], &position, simx_opmode_streaming);
		cout << position << endl;
	}
	system("pause");
}