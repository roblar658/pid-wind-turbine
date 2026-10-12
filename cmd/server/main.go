package main

import (
	"context"
	"encoding/json"
	"fmt"
	"html/template"
	"log"
	"math"
	"net/http"
	"os/exec"
	"runtime"
	"time"

	"wind-resistance-turbine/internal/turbine"
)

type SimulationData struct {
	Second       int     `json:"sec"`
	WindSpeed    float64 `json:"wind"`
	RotorRPM     float64 `json:"rpm"`
	TargetRPM    float64 `json:"targetRpm"`
	LoadROhms    float64 `json:"loadR"`
	CurrentAmps  float64 `json:"current"`
	TorqueNm     float64 `json:"torque"`
	PowerWatts   float64 `json:"power"`
}

type SimHardware struct {
	deltaP float64
	omega  float64
	loadR  float64
}

func (s *SimHardware) ReadPressure(ctx context.Context) (float64, error) { return s.deltaP, nil }
func (s *SimHardware) ReadRotorSpeed(ctx context.Context) (float64, error) { return s.omega, nil }
func (s *SimHardware) SetLoadResistance(ctx context.Context, r float64) error { s.loadR = r; return nil }

const dashboardHTML = `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Wind Turbine MPPT & Real 3D Environment</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <style>
        * { box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            background: #0b1120;
            color: #f1f5f9;
            margin: 0;
            padding: 16px;
        }
        .header {
            max-width: 1600px;
            margin: 0 auto 16px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            flex-wrap: wrap;
            gap: 12px;
        }
        .header h1 {
            margin: 0;
            font-size: 22px;
            color: #38bdf8;
            font-weight: 700;
        }
        .badges {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
        }
        .badge {
            background: #1e293b;
            border: 1px solid #334155;
            padding: 5px 12px;
            border-radius: 6px;
            font-size: 12px;
            font-weight: 600;
        }
        .dashboard-grid {
            max-width: 1600px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 16px;
        }
        @media (max-width: 1100px) {
            .dashboard-grid { grid-template-columns: 1fr; }
        }
        .panel {
            background: #131d35;
            border: 1px solid #1e293b;
            border-radius: 12px;
            padding: 16px;
            box-shadow: 0 10px 25px rgba(0,0,0,0.4);
        }
        .chart-box {
            position: relative;
            height: 285px;
            margin-bottom: 16px;
        }
        .chart-box:last-child {
            margin-bottom: 0;
        }
        .three-container {
            position: relative;
            height: 600px;
            border-radius: 8px;
            overflow: hidden;
            border: 1px solid #334155;
        }
        #threeCanvas {
            width: 100%;
            height: 100%;
            display: block;
        }
        .hud-overlay {
            position: absolute;
            top: 14px;
            left: 14px;
            background: rgba(15, 23, 42, 0.78);
            backdrop-filter: blur(8px);
            border: 1px solid rgba(255, 255, 255, 0.15);
            border-radius: 8px;
            padding: 12px 16px;
            font-size: 13px;
            line-height: 1.6;
            pointer-events: none;
        }
        .hud-overlay span {
            color: #38bdf8;
            font-weight: 700;
        }
        .controls {
            display: flex;
            align-items: center;
            gap: 12px;
            margin-top: 12px;
            background: #1e293b;
            padding: 8px 16px;
            border-radius: 8px;
        }
        .controls input[type="range"] {
            flex: 1;
            accent-color: #38bdf8;
        }
        .controls button {
            background: #0284c7;
            color: #fff;
            border: none;
            padding: 6px 14px;
            border-radius: 6px;
            font-weight: 600;
            cursor: pointer;
        }
        .controls button:hover {
            background: #0369a1;
        }
    </style>
</head>
<body>
    <div class="header">
        <div>
            <h1>Wind Turbine MPPT & Real 3D Environment</h1>
            <div style="color: #94a3b8; font-size: 13px; margin-top: 4px;">
                Atmospheric sky, tapered steel mast, aerodynamic nacelle, and variable resistance load telemetry.
            </div>
        </div>
        <div class="badges">
            <span class="badge" style="color: #38bdf8;">Blade Radius: 2.5 m</span>
            <span class="badge" style="color: #fbbf24;">Motor Kt=Ke: 1.8 NÂ·m/A</span>
            <span class="badge" style="color: #34d399;">Internal R: 0.4 Î©</span>
            <span class="badge" style="color: #f43f5e;">Reverse Lock: Passive</span>
        </div>
    </div>

    <div class="dashboard-grid">
        <div class="panel">
            <div class="chart-box">
                <canvas id="rpmChart"></canvas>
            </div>
            <div class="chart-box">
                <canvas id="elecChart"></canvas>
            </div>
        </div>

        <div class="panel">
            <div class="three-container">
                <canvas id="threeCanvas"></canvas>
                <div class="hud-overlay">
                    Time: <span id="hudTime">0s</span><br>
                    Wind Speed: <span id="hudWind">0.0 m/s</span><br>
                    Rotor Speed: <span id="hudRPM">0.0 RPM</span> (Target: <span id="hudTarget">0.0 RPM</span>)<br>
                    Electrical Power: <span id="hudPower">0 W</span><br>
                    Load Resistor: <span id="hudRes">0.0 Î©</span>
                </div>
            </div>
            <div class="controls">
                <button id="playBtn">Pause</button>
                <input type="range" id="timeSlider" min="0" max="160" value="0">
                <span id="sliderLabel" style="font-size: 13px; font-weight: 600; width: 45px;">0s</span>
            </div>
        </div>
    </div>

    <script>
        const simData = {{.}};
        const maxSec = simData.length - 1;

        const labels = simData.map(d => d.sec + 's');
        new Chart(document.getElementById('rpmChart').getContext('2d'), {
            type: 'line',
            data: {
                labels: labels,
                datasets: [
                    { label: 'Wind Speed (m/s)', data: simData.map(d => d.wind), borderColor: '#38bdf8', borderWidth: 2, yAxisID: 'yWind', tension: 0.2 },
                    { label: 'Rotor RPM', data: simData.map(d => d.rpm), borderColor: '#f59e0b', borderWidth: 2.5, yAxisID: 'yRPM', tension: 0.2 },
                    { label: 'MPPT Target RPM', data: simData.map(d => d.targetRpm), borderColor: '#ef4444', borderDash: [5, 4], borderWidth: 1.5, yAxisID: 'yRPM', pointRadius: 0 }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    x: { ticks: { color: '#94a3b8', maxTicksLimit: 10 }, grid: { color: '#1e293b' } },
                    yRPM: { position: 'left', title: { display: true, text: 'Rotor (RPM)', color: '#f59e0b' }, ticks: { color: '#f59e0b' }, grid: { color: '#1e293b' } },
                    yWind: { position: 'right', title: { display: true, text: 'Wind (m/s)', color: '#38bdf8' }, ticks: { color: '#38bdf8' }, grid: { drawOnChartArea: false } }
                },
                plugins: { legend: { labels: { color: '#f8fafc' } } }
            }
        });

        new Chart(document.getElementById('elecChart').getContext('2d'), {
            type: 'line',
            data: {
                labels: labels,
                datasets: [
                    { label: 'Power Harvested (W)', data: simData.map(d => d.power), borderColor: '#10b981', fill: true, backgroundColor: 'rgba(16, 185, 129, 0.15)', borderWidth: 2, yAxisID: 'yPower', tension: 0.2 },
                    { label: 'Current (A)', data: simData.map(d => d.current), borderColor: '#ec4899', borderWidth: 1.8, yAxisID: 'yCurrent', tension: 0.2 },
                    { label: 'Load Resistance (Î©)', data: simData.map(d => d.loadR > 50 ? 50 : d.loadR), borderColor: '#a855f7', borderDash: [4, 4], borderWidth: 1.5, yAxisID: 'yRes', pointRadius: 0 }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    x: { ticks: { color: '#94a3b8', maxTicksLimit: 10 }, grid: { color: '#1e293b' } },
                    yPower: { position: 'left', title: { display: true, text: 'Power (W)', color: '#10b981' }, ticks: { color: '#10b981' }, grid: { color: '#1e293b' } },
                    yCurrent: { position: 'right', title: { display: true, text: 'Current (A)', color: '#ec4899' }, ticks: { color: '#ec4899' }, grid: { drawOnChartArea: false } },
                    yRes: { position: 'right', title: { display: true, text: 'Load (Î©)', color: '#a855f7' }, ticks: { color: '#a855f7' }, grid: { drawOnChartArea: false } }
                },
                plugins: { legend: { labels: { color: '#f8fafc' } } }
            }
        });

        const canvas = document.getElementById('threeCanvas');
        const container = canvas.parentElement;
        const scene = new THREE.Scene();

        scene.background = new THREE.Color(0x7ec0ee);
        scene.fog = new THREE.FogExp2(0xcfe2f3, 0.022);

        const camera = new THREE.PerspectiveCamera(42, container.clientWidth / container.clientHeight, 0.1, 100);
        camera.position.set(0, 1.8, 8.8);

        const renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        renderer.shadowMap.enabled = true;

        const hemiLight = new THREE.HemisphereLight(0xffffff, 0x476930, 0.75);
        scene.add(hemiLight);

        const sunLight = new THREE.DirectionalLight(0xfffaed, 1.4);
        sunLight.position.set(6, 14, 8);
        sunLight.castShadow = true;
        scene.add(sunLight);

        const groundGeo = new THREE.PlaneGeometry(80, 80);
        const groundMat = new THREE.MeshStandardMaterial({
            color: 0x3d662c,
            roughness: 0.9,
            metalness: 0.05
        });
        const ground = new THREE.Mesh(groundGeo, groundMat);
        ground.rotation.x = -Math.PI / 2;
        ground.position.y = -2.5;
        ground.receiveShadow = true;
        scene.add(ground);

        const turbineGroup = new THREE.Group();
        turbineGroup.position.y = -2.5;
        scene.add(turbineGroup);

        const turbineWhite = new THREE.MeshStandardMaterial({
            color: 0xf8fafc,
            roughness: 0.35,
            metalness: 0.15
        });

        const towerGeo = new THREE.CylinderGeometry(0.12, 0.28, 5.8, 32);
        const tower = new THREE.Mesh(towerGeo, turbineWhite);
        tower.position.y = 2.9;
        tower.castShadow = true;
        turbineGroup.add(tower);

        const flangeGeo = new THREE.CylinderGeometry(0.33, 0.36, 0.16, 32);
        const flange = new THREE.Mesh(flangeGeo, new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.5 }));
        flange.position.y = 0.08;
        turbineGroup.add(flange);

        const nacelleGroup = new THREE.Group();
        nacelleGroup.position.set(0, 5.8, 0);
        turbineGroup.add(nacelleGroup);

        const nacelleBodyGeo = new THREE.CylinderGeometry(0.24, 0.27, 1.25, 24);
        nacelleBodyGeo.rotateX(Math.PI / 2);
        const nacelleBody = new THREE.Mesh(nacelleBodyGeo, turbineWhite);
        nacelleBody.position.z = -0.25;
        nacelleBody.castShadow = true;
        nacelleGroup.add(nacelleBody);

        const tailGeo = new THREE.ConeGeometry(0.24, 0.55, 24);
        tailGeo.rotateX(-Math.PI / 2);
        const tail = new THREE.Mesh(tailGeo, turbineWhite);
        tail.position.z = -0.92;
        nacelleGroup.add(tail);

        const sensorMast = new THREE.Mesh(
            new THREE.CylinderGeometry(0.015, 0.015, 0.26, 8),
            new THREE.MeshStandardMaterial({ color: 0x334155 })
        );
        sensorMast.position.set(0, 0.38, -0.6);
        nacelleGroup.add(sensorMast);

        const rotorGroup = new THREE.Group();
        rotorGroup.position.set(0, 0, 0.42);
        nacelleGroup.add(rotorGroup);

        const spinnerGeo = new THREE.ConeGeometry(0.26, 0.45, 32);
        spinnerGeo.rotateX(Math.PI / 2);
        const spinner = new THREE.Mesh(spinnerGeo, turbineWhite);
        spinner.castShadow = true;
        rotorGroup.add(spinner);

        function createRealBlade() {
            const bladeGroup = new THREE.Group();

            const root = new THREE.Mesh(
                new THREE.CylinderGeometry(0.045, 0.055, 0.3, 16),
                turbineWhite
            );
            root.position.y = 0.15;
            bladeGroup.add(root);

            const shape = new THREE.Shape();
            shape.moveTo(-0.04, 0);
            shape.lineTo(0.08, 0);
            shape.quadraticCurveTo(0.13, 0.9, 0.06, 2.1);
            shape.lineTo(0.02, 2.45);
            shape.lineTo(-0.02, 2.45);
            shape.quadraticCurveTo(-0.06, 1.1, -0.04, 0);

            const bladeGeo = new THREE.ExtrudeGeometry(shape, {
                depth: 0.025,
                bevelEnabled: true,
                bevelSegments: 2,
                steps: 1,
                bevelSize: 0.012,
                bevelThickness: 0.012
            });
            const bladeMesh = new THREE.Mesh(bladeGeo, turbineWhite);
            bladeMesh.position.set(0, 0.3, -0.012);
            bladeMesh.rotation.y = 0.12;
            bladeMesh.castShadow = true;
            bladeGroup.add(bladeMesh);

            const tip = new THREE.Mesh(
                new THREE.BoxGeometry(0.065, 0.3, 0.035),
                new THREE.MeshStandardMaterial({ color: 0xef4444, roughness: 0.3 })
            );
            tip.position.set(0.01, 2.45, 0);
            bladeGroup.add(tip);

            return bladeGroup;
        }

        for (let i = 0; i < 3; i++) {
            const blade = createRealBlade();
            blade.rotation.z = (i * 2 * Math.PI) / 3;
            rotorGroup.add(blade);
        }

        const windCount = 70;
        const windLines = [];
        const windMat = new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.35 });

        for (let i = 0; i < windCount; i++) {
            const points = [new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -1.2)];
            const lineGeo = new THREE.BufferGeometry().setFromPoints(points);
            const line = new THREE.Line(lineGeo, windMat);
            line.position.set(
                (Math.random() - 0.5) * 14,
                Math.random() * 7 - 1.5,
                Math.random() * 12 - 4
            );
            scene.add(line);
            windLines.push(line);
        }

        window.addEventListener('resize', () => {
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
        });

        let currentSecond = 0;
        let isPlaying = true;
        let lastTime = performance.now();

        const timeSlider = document.getElementById('timeSlider');
        const sliderLabel = document.getElementById('sliderLabel');
        const playBtn = document.getElementById('playBtn');

        playBtn.addEventListener('click', () => {
            isPlaying = !isPlaying;
            playBtn.textContent = isPlaying ? "Pause" : "Play";
        });

        timeSlider.addEventListener('input', (e) => {
            currentSecond = parseFloat(e.target.value);
            sliderLabel.textContent = Math.round(currentSecond) + 's';
        });

        function animate(now) {
            requestAnimationFrame(animate);
            const dt = (now - lastTime) / 1000;
            lastTime = now;

            if (isPlaying) {
                currentSecond += dt * 3.0;
                if (currentSecond > maxSec) currentSecond = 0;
                timeSlider.value = currentSecond;
                sliderLabel.textContent = Math.round(currentSecond) + 's';
            }

            const pt = simData[Math.min(Math.floor(currentSecond), maxSec)];
            const radPerSec = (pt.rpm * 2.0 * Math.PI) / 60.0;
            rotorGroup.rotation.z += radPerSec * dt;

            const windSpeedFactor = pt.wind * 0.8;
            for (let i = 0; i < windLines.length; i++) {
                const line = windLines[i];
                line.position.z += windSpeedFactor * dt;
                if (line.position.z > 6) {
                    line.position.z = -8;
                    line.position.x = (Math.random() - 0.5) * 14;
                    line.position.y = Math.random() * 7 - 1.5;
                }
            }

            document.getElementById('hudTime').textContent = pt.sec + 's';
            document.getElementById('hudWind').textContent = pt.wind + ' m/s';
            document.getElementById('hudRPM').textContent = pt.rpm + ' RPM';
            document.getElementById('hudTarget').textContent = pt.targetRpm + ' RPM';
            document.getElementById('hudPower').textContent = pt.power + ' W';
            document.getElementById('hudRes').textContent = pt.loadR + ' Î©';

            renderer.render(scene, camera);
        }

        requestAnimationFrame(animate);
    </script>
</body>
</html>`

func openBrowser(url string) {
    var cmd *exec.Cmd
    switch runtime.GOOS {
    case "windows":
        cmd = exec.Command("rundll32", "url.dll,FileProtocolHandler", url)
    case "darwin":
        cmd = exec.Command("open", url)
    default:
        cmd = exec.Command("xdg-open", url)
    }
    _ = cmd.Start()
}

func main() {
    cfg := turbine.MotorTurbineConfig{
        AirDensity:    1.225,
        RotorRadius:   2.5,
        RotorInertia:  35.0,
        OptTSR:        7.0,
        MaxCp:         0.44,
        MotorKm:       1.8,
        InternalR:     0.4,
        MinLoadR:      0.2,
        MaxLoadR:      200.0,
        MechFrictionB: 0.04,
    }
    sim := turbine.NewMotorSimulation(cfg, 5.0)
    hw := &SimHardware{omega: 5.0, loadR: 200.0}

    pid, err := turbine.NewPIDController(turbine.PIDConfig{
        Kp:        0.08,
        Ki:        0.02,
        Kd:        0.005,
        MinOutput: 1.0 / cfg.MaxLoadR,
        MaxOutput: 1.0 / cfg.MinLoadR,
    })
    if err != nil {
        log.Fatalf("Error configuring PID: %v", err)
    }

    controller := turbine.NewTurbineMotorController(hw, hw, hw, pid, sim)
    ctx := context.Background()

    durationSec := 160
    history := make([]SimulationData, 0, durationSec)
    startTime := time.Now()

    fmt.Println("Running wind turbine resistance simulation...")

    for s := 0; s <= durationSec; s++ {
        wind := 7.0 + 2.0*math.Sin(float64(s)*0.06)
        if s > 50 && s < 100 {
            wind += 3.5
        }

        hw.deltaP = 0.5 * cfg.AirDensity * wind * wind
        hw.omega = sim.Omega

        tNow := startTime.Add(time.Duration(s) * time.Second)
        windSpeed, targetOmega, currentOmega, loadR, _ := controller.Step(ctx, tNow)
        _, current, torque, power, _ := sim.Step(windSpeed, loadR, 1.0)

        history = append(history, SimulationData{
            Second:      s,
            WindSpeed:   math.Round(windSpeed*10) / 10,
            RotorRPM:    math.Round((currentOmega*60.0/(2.0*math.Pi))*10) / 10,
            TargetRPM:   math.Round((targetOmega*60.0/(2.0*math.Pi))*10) / 10,
            LoadROhms:   math.Round(loadR*100) / 100,
            CurrentAmps: math.Round(current*10) / 10,
            TorqueNm:    math.Round(torque*10) / 10,
            PowerWatts:  math.Round(power),
        })
    }

    jsonData, err := json.Marshal(history)
    if err != nil {
        log.Fatalf("JSON failed: %v", err)
    }

    tmpl := template.Must(template.New("dash").Parse(dashboardHTML))
    http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
        w.Header().Set("Content-Type", "text/html; charset=utf-8")
        _ = tmpl.Execute(w, template.JS(string(jsonData)))
    })

    port := "8082"
    url := "http://localhost:" + port
    fmt.Printf("\nDashboard running on: %s (Ctrl+C to stop)\n", url)

    go func() {
        time.Sleep(300 * time.Millisecond)
        openBrowser(url)
    }()

    if err := http.ListenAndServe(":"+port, nil); err != nil {
        log.Fatalf("Server error: %v", err)
    }
}
