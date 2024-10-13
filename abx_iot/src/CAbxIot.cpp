/*
 * CAbxIot.cpp
 *
 * Created on: 2024-10-13
 * Author: henry@abx
 */

#include <fstream>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <iostream>
#include <math.h>
#include "CAbxIot.h"



using namespace std;
using namespace ros;
using namespace Aws::Crt;

CAbxIot::CAbxIot()
{
	mBool = false;
	mInt = 0;
	mStr = "";

		mAbxPub = mNh.advertise<std_msgs::Header>("abx_iot", 10);

	mAbxSub = mNh.subscribe("abx_iot", 10, &CAbxIot::abxSubCallback, this);
	
	mAbxSrv = mNh.advertiseService("abx_iot", &CAbxIot::abxSrvCallback, this);

	mAbxCli = mNh.serviceClient<std_srvs::SetBool>("abx_iot");

	mAbxThread = new boost::thread(boost::bind(&CAbxIot::abxThreadFunc, this));

	mAbxTimer = mNh.createTimer(ros::Duration(1), boost::bind(&CAbxIot::abxTimerCallback, this));
}

CAbxIot::~CAbxIot()
{
	if (mAbxThread)
	{
		delete mAbxThread;
	}
}

void CAbxIot::abxSubCallback(const std_msgs::Header::ConstPtr& msg)
{
	ROS_INFO("Sub header freame_id: %s, seq: %d", msg->frame_id.data(), msg->seq);
	ROS_INFO_STREAM("Current stamp: " << msg->stamp);
}

bool CAbxIot::abxSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	ROS_INFO("Service Callback req.data: %d", req.data);

	res.success = true;
	res.message = "successful";

	return true;
}

void CAbxIot::abxThreadFunc()
{
	// Do the global initialization for the API.
	apiHandle.InitializeLogging(Aws::Crt::LogLevel::Debug, stderr);

	Aws::Crt::String input_endpoint = "a28rq6kqclvokz-ats.iot.ap-southeast-1.amazonaws.com";
	std::string input_cert = "/opt/abx/cert/abx-iot-thing-apse1-sn1-certificate.pem.crt";
	std::string input_key = "/opt/abx/cert/abx-iot-thing-apse1-sn1-private.pem.key";
	Aws::Crt::String input_clientId = "abx-iot-thing-apse1-sn1";
	uint32_t input_port = 0;
	Aws::Crt::String input_topic = "abx-iot-thing-apse1-sn1-topic1";
	uint32_t input_count = 10;
	Aws::Crt::String input_message = "This is test message from pi5";

	// Variables needed for the sample
    std::mutex receiveMutex;
    std::condition_variable receiveSignal;
    uint32_t receivedCount = 0;

	Aws::Iot::Mqtt5ClientBuilder *builder = Aws::Iot::Mqtt5ClientBuilder::NewMqtt5ClientBuilderWithMtlsFromPath(
        input_endpoint, input_cert.c_str(), input_key.c_str());

	// Check if the builder setup correctly.
    if (builder == nullptr)
    {
        printf(
            "Failed to setup mqtt5 client builder with error code %d: %s", LastError(), ErrorDebugString(LastError()));
        //return -1;
    }

	// Setup connection options
    std::shared_ptr<Mqtt5::ConnectPacket> connectOptions = std::make_shared<Mqtt5::ConnectPacket>();
    connectOptions->WithClientId(input_clientId);
    builder->WithConnectOptions(connectOptions);
    if (input_port != 0)
    {
        builder->WithPort(static_cast<uint32_t>(input_port));
    }

	std::promise<bool> connectionPromise;
    std::promise<void> stoppedPromise;
    std::promise<void> disconnectPromise;
    std::promise<bool> subscribeSuccess;

	// Setup lifecycle callbacks
    builder->WithClientConnectionSuccessCallback(
        [&connectionPromise](const Mqtt5::OnConnectionSuccessEventData &eventData) {
            fprintf(
                stdout,
                "Mqtt5 Client connection succeed, clientid: %s.\n",
                eventData.negotiatedSettings->getClientId().c_str());
            connectionPromise.set_value(true);
        });
    builder->WithClientConnectionFailureCallback([&connectionPromise](
                                                     const Mqtt5::OnConnectionFailureEventData &eventData) {
        fprintf(stdout, "Mqtt5 Client connection failed with error: %s.\n", aws_error_debug_str(eventData.errorCode));
        connectionPromise.set_value(false);
    });
    builder->WithClientStoppedCallback([&stoppedPromise](const Mqtt5::OnStoppedEventData &) {
        fprintf(stdout, "Mqtt5 Client stopped.\n");
        stoppedPromise.set_value();
    });
    builder->WithClientAttemptingConnectCallback([](const Mqtt5::OnAttemptingConnectEventData &) {
        fprintf(stdout, "Mqtt5 Client attempting connection...\n");
    });
    builder->WithClientDisconnectionCallback([&disconnectPromise](const Mqtt5::OnDisconnectionEventData &eventData) {
        fprintf(stdout, "Mqtt5 Client disconnection with reason: %s.\n", aws_error_debug_str(eventData.errorCode));
        disconnectPromise.set_value();
    });

	// This is invoked upon the receipt of a Publish on a subscribed topic.
    builder->WithPublishReceivedCallback(
        [&receiveMutex, &receivedCount, &receiveSignal](const Mqtt5::PublishReceivedEventData &eventData) {
            if (eventData.publishPacket == nullptr)
                return;

            std::lock_guard<std::mutex> lock(receiveMutex);
            ++receivedCount;
            fprintf(stdout, "Publish received on topic %s:", eventData.publishPacket->getTopic().c_str());
            fwrite(eventData.publishPacket->getPayload().ptr, 1, eventData.publishPacket->getPayload().len, stdout);
            fprintf(stdout, "\n");

            for (Mqtt5::UserProperty prop : eventData.publishPacket->getUserProperties())
            {
                fprintf(stdout, "\twith UserProperty:(%s,%s)\n", prop.getName().c_str(), prop.getValue().c_str());
            }
            receiveSignal.notify_all();
        });

	// Create Mqtt5Client
    std::shared_ptr<Aws::Crt::Mqtt5::Mqtt5Client> client = builder->Build();
	// Clean up the builder
    delete builder;

    if (client == nullptr)
    {
        fprintf(
            stdout, "Failed to Init Mqtt5Client with error code %d: %s", LastError(), ErrorDebugString(LastError()));
       // return -1;
    }

	// Start mqtt5 connection session
    if (client->Start())
    {
        if (connectionPromise.get_future().get() == false)
        {
			fprintf(stdout, "Failed to start client");
            //return -1;
        }

        auto onSubAck = [&subscribeSuccess](int error_code, std::shared_ptr<Mqtt5::SubAckPacket> suback) {
            if (error_code != 0)
            {
                fprintf(
                    stdout,
                    "MQTT5 Client Subscription failed with error code: (%d)%s\n",
                    error_code,
                    aws_error_debug_str(error_code));
                subscribeSuccess.set_value(false);
            }
            if (suback != nullptr)
            {
                for (Mqtt5::SubAckReasonCode reasonCode : suback->getReasonCodes())
                {
                    if (reasonCode > Mqtt5::SubAckReasonCode::AWS_MQTT5_SARC_UNSPECIFIED_ERROR)
                    {
                        fprintf(
                            stdout,
                            "MQTT5 Client Subscription failed with server error code: (%d)%s\n",
                            reasonCode,
                            suback->getReasonString()->c_str());
                        subscribeSuccess.set_value(false);
                        return;
                    }
                }
            }
            subscribeSuccess.set_value(true);
        };

		Mqtt5::Subscription sub1(input_topic, Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE);
        sub1.WithNoLocal(false);
        std::shared_ptr<Mqtt5::SubscribePacket> subPacket = std::make_shared<Mqtt5::SubscribePacket>();
        subPacket->WithSubscription(std::move(sub1));

		if (client->Subscribe(subPacket, onSubAck))
        {
            // Waiting for subscription completed.
            if (subscribeSuccess.get_future().get() == true)
			{
				fprintf(stdout, "Subscription Success.\n");
			}
			else
            {
                fprintf(stdout, "Subscription failed.\n");
            }
		}
		else
        {
            fprintf(stdout, "Subscribe operation failed on client.\n");
        }
	}

	/**
	 * Setup publish completion callback. The callback will get triggered when the publish completes (when
	 * the client received the PubAck from the server).
	 */
	auto onPublishComplete = [](int, std::shared_ptr<Aws::Crt::Mqtt5::PublishResult> result) {
		if (!result->wasSuccessful())
		{
			fprintf(stdout, "Publish failed with error_code: %d", result->getErrorCode());
		}
		else if (result != nullptr)
		{
			std::shared_ptr<Mqtt5::PubAckPacket> puback =
				std::dynamic_pointer_cast<Mqtt5::PubAckPacket>(result->getAck());
			if (puback->getReasonCode() == 0)
			{
				fprintf(stdout, "Publish Succeed.\n");
			}
			else
			{
				fprintf(
						stdout,
						"PubACK reason code: %d : %s\n",
						puback->getReasonCode(),
						puback->getReasonString()->c_str());
			}
		};
	};


	uint32_t publishedCount = 0;

	Rate rate(1);

	while (ok())
	{
#if 0
		std_msgs::Header msg;
		msg.seq = mInt++;
		msg.stamp = ros::Time::now();
		msg.frame_id = "abx";
		mAbxPub.publish(msg);

		std_srvs::SetBool srv;
		mBool = !mBool;
		srv.request.data = mBool;
		if (mAbxCli.call(srv))
		{
			mStr = srv.response.message;
			ROS_INFO("srv response success: %d, message:%s", srv.response.success, mStr.data());
		}
#else
		if (publishedCount < input_count)
		{
			// Add \" to 'JSON-ify' the message
			String message = "\"" + input_message + std::to_string(publishedCount + 1).c_str() + "\"";
			ByteCursor payload = ByteCursorFromString(message);

			std::shared_ptr<Mqtt5::PublishPacket> publish = std::make_shared<Mqtt5::PublishPacket>(
					input_topic, payload, Mqtt5::QOS::AWS_MQTT5_QOS_AT_LEAST_ONCE);
			if (client->Publish(publish, onPublishComplete))
			{
				++publishedCount;
			}

			//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

#endif

		rate.sleep();
	}

	{
		std::unique_lock<std::mutex> receivedLock(receiveMutex);
		receiveSignal.wait(receivedLock, [&] { return receivedCount >= input_count; });
	}

	// Unsubscribe from the topic.
	std::promise<void> unsubscribeFinishedPromise;
	std::shared_ptr<Mqtt5::UnsubscribePacket> unsub = std::make_shared<Mqtt5::UnsubscribePacket>();
	unsub->WithTopicFilter(input_topic);
	if (!client->Unsubscribe(unsub, [&](int, std::shared_ptr<Mqtt5::UnSubAckPacket>) {
				unsubscribeFinishedPromise.set_value();
				}))
	{
		fprintf(stdout, "Unsubscription failed.\n");
		exit(-1);
	}
	unsubscribeFinishedPromise.get_future().wait();

	// Disconnect
	if (!client->Stop())
	{
		fprintf(stdout, "Failed to disconnect from the mqtt connection. Exiting..\n");
		//return -1;
	}

	stoppedPromise.get_future().wait();
}

void CAbxIot::abxTimerCallback()
{
	ROS_WARN("Timer Callback");
}


// EOF
