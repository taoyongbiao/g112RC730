#include <windows.h>
#include <stdio.h>

#pragma pack(push, 4)
struct FrameSuspensionData
{
	float height;
	float velocity;
};

struct FrameTyreData
{
	float slipRatio;
	float slipAngle;
	float load;
	float rotationRate;
};

struct FrameData
{
	float clock;
	float throttle;
	float brake;
	int gear;
	float rpm;
	float steering;
	float speed;

	float velocity[3];
	float acceleration[3];

	float yaw;
	float pitch;
	float roll;

	float comy;

	FrameTyreData tyreData[4];
	FrameSuspensionData suspensionData[4];

	float angular[3];

	unsigned int extraMessage;
	unsigned int running;
};
#pragma pack(pop)

typedef void(*RCInit)(bool);
typedef void(*RCEnd)();
typedef void(*RCReadFrameData)(FrameData& msg);


void main()
{

	auto handler = LoadLibraryA("RacingRemoteController.dll");
	if (handler == 0)
	{
		system("pause");
		return;
	}

	RCInit fInit = (RCInit)GetProcAddress(handler, "RCInit");
	RCReadFrameData fRead = (RCReadFrameData)GetProcAddress(handler, "RCReadFrameData");
	RCEnd fEnd = (RCEnd)GetProcAddress(handler, "RCEnd");

retry:
	while (true)
	{
		try
		{
			fInit(false);
			break;
		}
		catch (...)
		{
			printf("rc is not ready yet\n");
			Sleep(1000);
		}
	}

	FrameData msg;
	while (true)
	{
		try
		{
			fRead(msg);
			if (msg.running == 0)
			{
				printf("not any data!\n");
				Sleep(1000);
				continue;
			}
			if (msg.running == 2)
			{
				fEnd();
				printf("rc is finish\n");
				goto retry;
			}
			printf("clock:%.5f, speed:%.5f \n", msg.clock, msg.speed);
			printf("throttle:%.5f, brake:%.5f, gear:%d, rpm:%.5f, steering: %.5f\n", msg.throttle, msg.brake, msg.gear, msg.rpm, msg.steering);
			printf("velocity: x:%.5f, y:%.5f, z:%f \n", msg.velocity[0], msg.velocity[1], msg.velocity[2]);
			printf("acceleration: x:%.5f, y:%.5f, z:%.5f \n", msg.acceleration[0], msg.acceleration[1], msg.acceleration[2]);
			printf("comy: %.5f \n", msg.comy);
			printf("angular: x:%.5f, y:%.5f, z:%.5f \n", msg.angular[0], msg.angular[1], msg.angular[2]);
			printf("load: RL:%.5f, RR:%.5f, FL:%.5f, FR:%.5f \n", msg.tyreData[0].load, msg.tyreData[1].load, msg.tyreData[2].load, msg.tyreData[3].load);
			printf("slipRatio: RL:%.5f, RR:%.5f, FL:%.5f, FR:%.5f \n", msg.tyreData[0].slipRatio, msg.tyreData[1].slipRatio, msg.tyreData[2].slipRatio, msg.tyreData[3].slipRatio);
			printf("slipAngle: RL:%.5f, RR:%.5f, FL:%.5f, FR:%.5f \n", msg.tyreData[0].slipAngle, msg.tyreData[1].slipAngle, msg.tyreData[2].slipAngle, msg.tyreData[3].slipAngle);
			printf("rotationRate: RL:%.5f, RR:%.5f, FL:%.5f, FR:%.5f \n", msg.tyreData[0].rotationRate, msg.tyreData[1].rotationRate, msg.tyreData[2].rotationRate, msg.tyreData[3].rotationRate);

			printf("suspension height: RL:%.5f, RR:%.5f, FL:%.5f, FR:%.5f \n", msg.suspensionData[0].height, msg.suspensionData[1].height, msg.suspensionData[2].height, msg.suspensionData[3].height);
			printf("suspension velocity: RL:%.5f, RR:%.5f, FL:%.5f, FR:%.5f \n", msg.suspensionData[0].velocity, msg.suspensionData[1].velocity, msg.suspensionData[2].velocity, msg.suspensionData[3].velocity);


			printf("roll:%.5f, pitch:%.5f,yaw:%.5f \n", msg.roll, msg.pitch, msg.yaw);
			Sleep(100);
		}
		catch (...)
		{
			system("read failed");
			return;
		}
	}

}
