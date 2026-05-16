"use client";

import React, { useEffect, useState, useRef } from "react";

// Types
type Telemetry = {
  battery: number;
  altitude: number;
  phase: string;
  speed: number;
  gps: string;
  signal: number;
};

type Defect = {
  class: string;
  confidence: number;
  bbox: number[];
  severity: string;
  timestamp: number;
  frame_id: number;
};

type WSData = {
  telemetry: Telemetry;
  defects: Defect[];
  timestamp: number;
};

export default function Dashboard() {
  const [telemetry, setTelemetry] = useState<Telemetry>({
    battery: 100,
    altitude: 0.0,
    phase: "OFFLINE",
    speed: 0.0,
    gps: "--",
    signal: 0,
  });
  const [defects, setDefects] = useState<Defect[]>([]);
  const [sysTime, setSysTime] = useState<string>("00:00:00");
  const [logs, setLogs] = useState<string[]>(["> SYS.INIT() OK..."]);
  const logsEndRef = useRef<HTMLDivElement>(null);
  const lastPhaseRef = useRef<string>("OFFLINE");

  // System clock
  useEffect(() => {
    const timer = setInterval(() => {
      setSysTime(new Date().toISOString().split("T")[1].split(".")[0]);
    }, 1000);
    return () => clearInterval(timer);
  }, []);

  // Auto-scroll logs
  useEffect(() => {
    logsEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [logs]);

  // WebSocket Connection
  useEffect(() => {
    const wsUrl = "ws://localhost:8000/ws/telemetry";
    let ws: WebSocket;
    
    const connect = () => {
      ws = new WebSocket(wsUrl);
      
      ws.onopen = () => {
        setLogs(prev => [...prev, "> DATA.LINK ESTABLISHED"]);
      };

      ws.onmessage = (event) => {
        try {
          const data: WSData = JSON.parse(event.data);
          
          if (data.telemetry.phase !== lastPhaseRef.current) {
            setLogs(prev => [...prev, `> PHASE.TRANSITION: ${data.telemetry.phase}`]);
            lastPhaseRef.current = data.telemetry.phase;
          }

          if (data.defects.length > 0) {
            const highSev = data.defects.some(d => d.severity === "CRITICAL");
            if (highSev && !defects.some(d => d.severity === "CRITICAL")) {
               setLogs(prev => [...prev, "> WARNING: CRITICAL ANOMALY SCANNED"]);
            }
          }

          setTelemetry(data.telemetry);
          setDefects(data.defects);
        } catch (e) {
          console.error("Parse error", e);
        }
      };

      ws.onclose = () => {
        setTelemetry(prev => ({ ...prev, phase: "LINK_LOST" }));
        setLogs(prev => [...prev, "> ERROR: DATA.LINK SEVERED. RECONNECTING..."]);
        setTimeout(connect, 2000);
      };
    };

    connect();

    return () => {
      if (ws) ws.close();
    };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  // Helper for battery color
  const getBatteryColor = (level: number) => {
    if (level > 60) return "text-zinc-300";
    if (level > 20) return "text-yellow-500";
    return "text-red-500 animate-pulse";
  };

  return (
    <main className="min-h-screen bg-black text-zinc-300 p-4 lg:p-6 font-mono flex flex-col gap-4">
      
      {/* --- HEADER --- */}
      <header className="flex justify-between items-end border-b border-zinc-800 pb-4">
        <div>
          <h1 className="text-xl font-bold tracking-[0.2em] text-white">S.T.R.I.D.E.</h1>
          <p className="text-xs text-zinc-500 mt-1">v2.0.4 // AUTONOMOUS_INSPECTION_ECOSYSTEM</p>
        </div>
        <div className="text-right">
          <div className="flex items-center gap-3 justify-end mb-1">
            <span className="text-xs tracking-widest text-zinc-500">SYS.CLOCK</span>
            <span className="text-sm tracking-wider">{sysTime} UTC</span>
          </div>
          <div className="flex items-center gap-2 justify-end">
             <div className={`w-2 h-2 rounded-full ${telemetry.phase === "LINK_LOST" || telemetry.phase === "OFFLINE" ? "bg-red-500" : "bg-emerald-500 shadow-[0_0_8px_rgba(16,185,129,0.8)]"}`}></div>
             <span className="text-xs font-bold tracking-widest text-white">
                {telemetry.phase === "LINK_LOST" || telemetry.phase === "OFFLINE" ? "OFFLINE" : "LIVE"}
             </span>
          </div>
        </div>
      </header>

      {/* --- MAIN GRID --- */}
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 flex-1 min-h-0">
        
        {/* LEFT PANEL: TELEMETRY */}
        <section className="lg:col-span-3 flex flex-col gap-6">
          <div className="border border-zinc-800 p-4 bg-zinc-950/50">
            <h2 className="text-xs tracking-widest text-zinc-500 mb-4 uppercase border-b border-zinc-800 pb-2">Primary_Telemetry</h2>
            
            <div className="space-y-4 text-sm">
              <div className="flex justify-between items-center">
                <span className="text-zinc-500">PHASE</span>
                <span className={`tracking-widest font-bold ${telemetry.phase.includes('INSPECT') ? 'text-blue-400' : 'text-white'}`}>
                  {telemetry.phase}
                </span>
              </div>
              
              <div className="flex justify-between items-center">
                <span className="text-zinc-500">ALTITUDE</span>
                <span>{telemetry.altitude.toFixed(2)} M</span>
              </div>
              
              <div className="flex justify-between items-center">
                <span className="text-zinc-500">VELOCITY</span>
                <span>{telemetry.speed.toFixed(1)} M/S</span>
              </div>
              
              <div className="flex justify-between items-center">
                <span className="text-zinc-500">LOC.COORD</span>
                <span className="text-xs tracking-tighter">{telemetry.gps}</span>
              </div>
              
              <div className="flex justify-between items-center pt-2">
                <span className="text-zinc-500">BATT.PWR</span>
                <span className={`font-bold ${getBatteryColor(telemetry.battery)}`}>
                  {telemetry.battery}%
                </span>
              </div>
              {/* Battery bar */}
              <div className="w-full bg-zinc-900 h-1 mt-1">
                <div 
                  className={`h-full ${telemetry.battery > 20 ? 'bg-white' : 'bg-red-500'}`} 
                  style={{ width: `${telemetry.battery}%` }}
                />
              </div>
            </div>
          </div>

          <div className="border border-zinc-800 p-4 bg-zinc-950/50 flex-1 flex flex-col">
            <h2 className="text-xs tracking-widest text-zinc-500 mb-4 uppercase border-b border-zinc-800 pb-2">Sys_Diagnostics</h2>
            <div className="space-y-3 text-xs">
              <div className="flex justify-between">
                <span>COM.SIGNAL</span>
                <span className={telemetry.signal > 80 ? "text-emerald-500" : "text-yellow-500"}>{telemetry.signal} dBm</span>
              </div>
              <div className="flex justify-between">
                <span>AI.INFERENCE</span>
                <span className="text-emerald-500">NOMINAL</span>
              </div>
              <div className="flex justify-between">
                <span>LIDAR.ARRAY</span>
                <span className="text-emerald-500">ACTIVE</span>
              </div>
              <div className="flex justify-between">
                <span>OBJ.AVOID</span>
                <span className="text-emerald-500">STANDBY</span>
              </div>
            </div>
          </div>
        </section>

        {/* CENTER PANEL: VIDEO FEED */}
        <section className="lg:col-span-6 flex flex-col">
          <div className="border border-zinc-800 bg-zinc-950/50 relative flex-1 flex items-center justify-center p-1">
             {/* Reticle Accents */}
             <div className="absolute top-0 left-0 w-4 h-4 border-t-2 border-l-2 border-zinc-500 m-2"></div>
             <div className="absolute top-0 right-0 w-4 h-4 border-t-2 border-r-2 border-zinc-500 m-2"></div>
             <div className="absolute bottom-0 left-0 w-4 h-4 border-b-2 border-l-2 border-zinc-500 m-2"></div>
             <div className="absolute bottom-0 right-0 w-4 h-4 border-b-2 border-r-2 border-zinc-500 m-2"></div>
             
             {/* eslint-disable-next-line @next/next/no-img-element */}
             <img 
               src="http://localhost:8000/video_feed" 
               alt="Drone POV Stream" 
               className="w-full h-full object-cover max-h-[600px] grayscale-[20%] contrast-125 brightness-90"
             />

             {/* Connection Error Overlay */}
             {(telemetry.phase === "OFFLINE" || telemetry.phase === "LINK_LOST") && (
                <div className="absolute inset-0 bg-black/80 flex flex-col items-center justify-center">
                   <div className="text-red-500 mb-2 font-bold tracking-widest text-lg">AWAITING VIDEO STREAM</div>
                   <div className="text-xs text-zinc-500 uppercase tracking-widest">Run `python backend/main.py`</div>
                </div>
             )}
          </div>
        </section>

        {/* RIGHT PANEL: LOGS & DEFECTS */}
        <section className="lg:col-span-3 flex flex-col gap-6">
          
          <div className="border border-zinc-800 p-4 bg-zinc-950/50 flex flex-col flex-1 min-h-[200px]">
            <h2 className="text-xs tracking-widest text-zinc-500 mb-4 uppercase border-b border-zinc-800 pb-2 flex justify-between">
              <span>Event_Log</span>
              <span className="text-[10px]">C:\SYS\LOG</span>
            </h2>
            <div className="overflow-y-auto flex-1 space-y-2 text-xs font-mono pr-2">
              {logs.map((log, i) => (
                <div key={i} className={log.includes("ERROR") || log.includes("CRITICAL") ? "text-red-400" : log.includes("WARNING") ? "text-yellow-400" : "text-zinc-400"}>
                  {log}
                </div>
              ))}
              <div ref={logsEndRef} />
            </div>
          </div>

          <div className="border border-zinc-800 p-4 bg-zinc-950/50 min-h-[250px]">
            <h2 className="text-xs tracking-widest text-zinc-500 mb-4 uppercase border-b border-zinc-800 pb-2 flex justify-between items-center">
              <span>Anomaly_Registry</span>
              <span className="bg-zinc-800 text-white px-2 py-0.5 rounded text-[10px]">{defects.length} FOUND</span>
            </h2>
            
            <div className="space-y-3 overflow-y-auto max-h-[200px]">
              {defects.length === 0 ? (
                <div className="text-xs text-zinc-600 text-center py-8 border border-dashed border-zinc-800">
                  NO ANOMALIES DETECTED
                </div>
              ) : (
                defects.map((d, i) => (
                  <div key={i} className="border border-zinc-800 p-2 bg-black text-xs">
                    <div className="flex justify-between items-start mb-1">
                      <span className={`font-bold ${d.severity === 'CRITICAL' ? 'text-red-500' : d.severity === 'WARNING' ? 'text-yellow-500' : 'text-blue-400'}`}>
                        [{d.severity}] {d.class.toUpperCase()}
                      </span>
                      <span className="text-zinc-500">{(d.confidence * 100).toFixed(1)}%</span>
                    </div>
                    <div className="text-[10px] text-zinc-600">
                      T+{Math.floor(d.timestamp % 10000)}s | FRM: {d.frame_id}
                    </div>
                  </div>
                ))
              )}
            </div>
          </div>

        </section>
      </div>
    </main>
  );
}
