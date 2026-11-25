# Unity UDP Ros Bridge
 
There are 4 main ROS nodes that handle communication. Each receives JSON over UDP and converts it into the corresponding ROS function

- `udp_bridge_pub`: publishes the recieved data onto a ROS topic
- `udp_bridge_sub`: subscribes to a ROS topic, and sends results back to Unity over UDP
- `udp_bridge_srv`: sends a ROS service request and sends the response back to Unity over UDP
- `udp_bridge_action`: send a ROS action request, and sends the feedback and results to Unity over UDP.

A launch script called `startBridges.sh` exists to launch all of the bridges at once.

## Ports

Each part of the bridge uses a different UDP port on localhost (`127.0.0.1`)

- `65434`: input for `udp_bridge_pub` (Unity to ROS)
- `65435`: output for `udp_bridge_sub` (ROS to Unity)
- `65436`: input for `udp_bridge_sub` configuration (Unity to ROS)
- `65437`: input for `udp_bridge_srv` (Unity to ROS)
- `65438`: output for `udp_bridge_srv` (ROS to Unity)
- `65439`: input for `udp_bridge_action`. (Unity to ROS)
- `65440`: output for `udp_bridge_action`. (ROS to Unity)

## Publishing

`PublishMessage(string message)` takes in a string containing JSON and sends it to ROS for `udp_bridge_pub` to break down and handle. Publishers are created lazily the first time the bridge sees a topic.

JSON should be in the following format:

```json
{
  "topic": "/chatter",
  "msgType": "std_msgs/msg/String",
  "data": {
    "data": "Hello, World!"
  }
}
```

## Subscription

`ConfigureSubscription(string topic, string msgType)` takes in two strings, the topic and the message type. **No JSON is required when creating a subscription.**

Whenever new information is published to the topic, `udp_bridge_sub` will serialize it into JSON and send it to Unity. `GetLatestMessage(string topic)` will return the latest message for the specified topic. The JSON returned from `GetLatestMessage` will be in the following format.

```json
{
  "topic": "/chatter",
  "msgType": "std_msgs/msg/String",
  "data": {
    "data": "Hello back!"
  }
}
```

## Services

`PublishClientReq(string message, string serviceKey)` is one of two ways you make requests to ROS services. `message` is a string containing JSON in the below format. The `serviceKey` field is optional, and will be set to the service field of `message` if not supplied. `PublishClientReq` will wait for the service response and then return it, or return an empty JSON object if the service takes
too long to respond. 

`SendClientReq(string message)` is the other way to make requests to a ROS service. It takes in a string containing JSON in the below format and sends it to `udp_bridge_srv`. It will not wait for the response. To check the response, use the `GetLatestServiceResponse(string serviceKey)`.

The request JSON should be in the following format

```json
{
  "service": "/set_bool",
  "srvType": "std_srvs/srv/SetBool",
  "request": {
    "data": true
  }
}
```

The response JSON will be in the following format

```json
{
  "service": "/set_bool",
  "response": {
    "success": true,
  }
}
```

## Actions

`PublishActionReq(string message, string actionName)` is the function for making an action request. It's very similar to `PublishClientReq`, except it doesn't return the result (because actions have multiple responses). The `message` is a string containing JSON in the following format, `actionName` is an optional field that will be set to the `action` field of the JSON if not supplied.

```json
{
  "action": "fibonacci",
  "actionType": "action/Fibonacci",
  "goal": {
    "order": 6
  }
}
```

To get the latest feedback and/or results of the action, use the `GetLatestActionFeedback(string actionName)` function. `actionName` is the same as the `action` field of the request JSON. It will return JSON in one of the two following formats.

For feedback during the action

```json
{
  "action": "fibonacci",
  "feedback": {
    "_partial_sequence": [1, 1, 2]
  }
}
```

For the result when the action is completed

```json
{
  "action": "fibonacci",
  "result": {
    "_sequence": [1, 1, 2, 3, 5, 8],
  }
}
```

_This documentation was last updated on November 14th, 2026_