import ROSLIB from "roslib";

export function createRosConnection(url: string): ROSLIB.Ros {
  return new ROSLIB.Ros({ url });
}

export function createTopic(ros: ROSLIB.Ros, name: string, messageType: string): ROSLIB.Topic {
  return new ROSLIB.Topic({
    ros,
    name,
    messageType,
    queue_size: 10
  });
}

export function createService(ros: ROSLIB.Ros, name: string, serviceType: string): ROSLIB.Service {
  return new ROSLIB.Service({
    ros,
    name,
    serviceType
  });
}
