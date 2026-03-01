import { useEffect, useMemo, useState } from "react";
import ROSLIB from "roslib";
import { createRosConnection, createService, createTopic } from "./lib/ros";
import type { Severity, UserMessage } from "./types";

type ConnectionState = "CONNECTING" | "CONNECTED" | "DISCONNECTED";

const DEFAULT_ROSBRIDGE_URL =
  import.meta.env.VITE_ROSBRIDGE_URL ?? "ws://localhost:9090";

function severityFromCode(value: number): Severity {
  if (value === 2) {
    return "ERROR";
  }
  if (value === 1) {
    return "WARNING";
  }
  return "INFO";
}

export default function App() {
  const [connectionState, setConnectionState] = useState<ConnectionState>("CONNECTING");
  const [mode, setMode] = useState("UNKNOWN");
  const [gaitLabel, setGaitLabel] = useState("UNKNOWN");
  const [gaitContinuous, setGaitContinuous] = useState("0.00");
  const [gaitConfidence, setGaitConfidence] = useState("0.00");
  const [calibrationStatus, setCalibrationStatus] = useState("UNKNOWN");
  const [calibrationDetail, setCalibrationDetail] = useState("-");
  const [serviceStatus, setServiceStatus] = useState("-");
  const [messages, setMessages] = useState<UserMessage[]>([]);

  const ros = useMemo(() => createRosConnection(DEFAULT_ROSBRIDGE_URL), []);

  useEffect(() => {
    const onConnection = () => setConnectionState("CONNECTED");
    const onClose = () => setConnectionState("DISCONNECTED");
    const onError = () => setConnectionState("DISCONNECTED");

    ros.on("connection", onConnection);
    ros.on("close", onClose);
    ros.on("error", onError);
    setConnectionState((ros as unknown as { isConnected?: boolean }).isConnected ? "CONNECTED" : "CONNECTING");

    const modeTopic = createTopic(ros, "/exo/system/mode", "std_msgs/String");
    const calibrationTopic = createTopic(
      ros,
      "/exo/system/calibration_status",
      "exo_interfaces/msg/CalibrationStatus"
    );
    const userMessageTopic = createTopic(
      ros,
      "/exo/system/user_message",
      "exo_interfaces/msg/UserMessage"
    );
    const gaitTopic = createTopic(ros, "/exo/gait/phase", "exo_interfaces/msg/GaitPhase");

    modeTopic.subscribe((msg: { data: string }) => {
      setMode(msg.data);
    });

    calibrationTopic.subscribe((msg: { status: number; detail: string }) => {
      const statusMap: Record<number, string> = {
        0: "UNKNOWN",
        1: "MISSING",
        2: "VALID",
        3: "INVALID"
      };
      setCalibrationStatus(statusMap[msg.status] ?? "UNKNOWN");
      setCalibrationDetail(msg.detail || "-");
    });

    userMessageTopic.subscribe((msg: { severity: number; code: string; text: string; header?: { stamp?: { sec?: number } } }) => {
      const entry: UserMessage = {
        severity: severityFromCode(msg.severity),
        code: msg.code,
        text: msg.text,
        stampSec: msg.header?.stamp?.sec ?? 0
      };
      setMessages((prev) => [entry, ...prev].slice(0, 20));
    });

    gaitTopic.subscribe((msg: { phase_label: number; phase_continuous: number; confidence: number }) => {
      const phaseMap: Record<number, string> = {
        0: "STANCE",
        1: "SWING",
        2: "TRANSITION"
      };
      setGaitLabel(phaseMap[msg.phase_label] ?? "UNKNOWN");
      setGaitContinuous(msg.phase_continuous.toFixed(2));
      setGaitConfidence(msg.confidence.toFixed(2));
    });

    const connectionWatchdog = window.setInterval(() => {
      const connected = Boolean((ros as unknown as { isConnected?: boolean }).isConnected);
      setConnectionState((prev) => {
        if (connected && prev !== "CONNECTED") {
          return "CONNECTED";
        }
        if (!connected && prev === "CONNECTED") {
          return "DISCONNECTED";
        }
        return prev;
      });
    }, 500);

    return () => {
      modeTopic.unsubscribe();
      calibrationTopic.unsubscribe();
      userMessageTopic.unsubscribe();
      gaitTopic.unsubscribe();
      window.clearInterval(connectionWatchdog);
      ros.close();
    };
  }, [ros]);

  function callSetMode(requestedMode: string) {
    const service = createService(ros, "/exo/system/set_mode", "exo_interfaces/srv/SetOperatingMode");
    const request = new ROSLIB.ServiceRequest({ mode: requestedMode });
    service.callService(request, (result: { accepted: boolean; message: string }) => {
      setServiceStatus(`${requestedMode}: ${result.accepted ? "accepted" : "rejected"} (${result.message})`);
    });
  }

  function callTrigger(serviceName: string, label: string) {
    const service = createService(ros, serviceName, "std_srvs/srv/Trigger");
    const request = new ROSLIB.ServiceRequest({});
    service.callService(request, (result: { success: boolean; message: string }) => {
      setServiceStatus(`${label}: ${result.success ? "ok" : "failed"} (${result.message})`);
    });
  }

  return (
    <div className="app-shell">
      <header className="hero">
        <h1>Exoskeleton Operator Dashboard</h1>
        <div className={`pill ${connectionState.toLowerCase()}`}>{connectionState}</div>
      </header>

      <section className="grid">
        <article className="card">
          <h2>System State</h2>
          <p><strong>Mode:</strong> {mode}</p>
          <p><strong>Calibration:</strong> {calibrationStatus}</p>
          <p><strong>Detail:</strong> {calibrationDetail}</p>
          <p><strong>Gait Estimate:</strong> {gaitLabel} (phase={gaitContinuous}, confidence={gaitConfidence})</p>
          <p><strong>Last Service:</strong> {serviceStatus}</p>
        </article>

        <article className="card">
          <h2>Mode Controls</h2>
          <div className="button-row">
            <button onClick={() => callTrigger("/exo/system/start_unit", "start_unit")}>Start Unit</button>
            <button onClick={() => callTrigger("/exo/system/shutdown_unit", "shutdown_unit")}>Shutdown Unit</button>
            <button onClick={() => callSetMode("IDLE")}>Set IDLE</button>
            <button onClick={() => callSetMode("CALIBRATION")}>Set CALIBRATION</button>
            <button onClick={() => callSetMode("ASSISTIVE")}>Set ASSISTIVE</button>
            <button onClick={() => callSetMode("FAULT")}>Set FAULT</button>
          </div>
        </article>

        <article className="card">
          <h2>Simulation Controls</h2>
          <div className="button-row">
            <button onClick={() => callTrigger("/exo/system/start_simulation", "start_simulation")}>Start Simulation Walk</button>
            <button onClick={() => callTrigger("/exo/system/stop_simulation", "stop_simulation")}>Stop Simulation Walk</button>
          </div>
        </article>

        <article className="card">
          <h2>Calibration Controls</h2>
          <div className="button-row">
            <button onClick={() => callTrigger("/exo/system/run_full_calibration", "run_full_calibration")}>Run Full Calibration + Validate</button>
            <button onClick={() => callTrigger("/exo/system/validate_calibration", "validate_calibration")}>Validate Calibration</button>
          </div>
        </article>

        <article className="card full-width">
          <h2>User Messages</h2>
          <div className="message-list">
            {messages.length === 0 ? (
              <p className="muted">No messages received yet.</p>
            ) : (
              messages.map((message, index) => (
                <div key={`${message.stampSec}-${index}`} className={`message ${message.severity.toLowerCase()}`}>
                  <span className="code">[{message.code}]</span>
                  <span>{message.text}</span>
                </div>
              ))
            )}
          </div>
        </article>
      </section>
    </div>
  );
}
