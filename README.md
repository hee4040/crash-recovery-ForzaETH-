<h1 align="center">🏎️ F1TENTH Collision Recovery with TinyLidarNet</h1>

<p align="center">
  <b>Pure Pursuit 주행 중 충돌 발생 시</b><br/>
  <b>사람의 개입 없이 자동 복구하여 주행을 재개하는 시스템</b>
</p>

<br/>

### 📌 Project Overview
<hr/>

<p>
본 프로젝트는 <b>F1TENTH 자율주행 대회</b> 환경에서 주행 중 충돌이 발생했을 때,
운전자의 개입 없이 차량이 스스로 복구하여 주행을 재개하도록 설계된 시스템이다.
</p>

<p>
기존 <b>Pure Pursuit(PP)</b> 기반 주행 중 충돌이 발생하면,
<b>TinyLidarNet(TLN)</b>을 이용한 후진 주행으로 차량을 안전하게 탈출시키고,
<b>Particle Filter</b>를 통해 포즈 안정화를 확인한 후 다시 PP 주행으로 복귀한다.
</p>

<p>
본 프로젝트는 <b>ForzaETH 시뮬레이션 환경</b>을 기반으로 하며,
기존 ForzaETH 코드 중 일부를 수정 및 확장하여 <code>src/</code> 폴더에 구현하였다.
</p>

<br/>

### 🎯 Objectives
<hr/>

<ul>
  <li>충돌 이후 <b>수동 개입 없는 주행 복구</b></li>
  <li>PP 주행의 한계를 보완하는 <b>학습 기반 후진 전략</b></li>
  <li>State Machine 기반 <b>안정적인 제어 전환</b></li>
  <li>대회 환경을 고려한 <b>엔지니어링 중심 시스템 설계</b></li>
</ul>

<br/>

### 🧠 System Architecture
<hr/>

<p align="center">
  <img width="710" height="362" alt="image" src="https://github.com/user-attachments/assets/29c0d9c1-8dd6-4cab-bba4-7f4bf36bf288" />
</p>

<pre>
Pure Pursuit 주행
        ↓
Collision Detector
        ↓
State Machine
        ↓
TinyLidarNet Inference (Backward)
        ↓
Backward Done
        ↓
Particle Filter 안정화 확인
        ↓
Pure Pursuit 주행 복귀
</pre>

<br/>

### 🔧 Node Descriptions
<hr/>

#### /collision_detector_node
<ul>
  <li>차량 센서 기반 충돌 감지</li>
  <li>충돌 발생 시 <code>/collision_detected</code> 토픽 발행</li>
</ul>

---

#### /state_machine
<ul>
  <li>전체 시스템의 중앙 제어 노드</li>
  <li>충돌, 후진 완료, 포즈 안정화 상태 종합 판단</li>
  <li>현재 상태를 <code>/state</code> 토픽으로 발행</li>
</ul>

<b>State Example</b>
<ul>
  <li>PP_RUNNING</li>
  <li>COLLISION_DETECTED</li>
  <li>TLN_BACKWARD</li>
  <li>POSE_RECOVERY</li>
  <li>PP_RECOVERED</li>
</ul>

---

#### /tln_inference
<ul>
  <li>TinyLidarNet 기반 후진 주행 추론</li>
  <li>충돌 이후 안전한 탈출 경로 생성</li>
  <li>후진 완료 시 <code>/tln/backward_done</code> 발행</li>
</ul>

---

#### /particle_filter
<ul>
  <li>후진 이후 차량의 위치 추정(Localization)</li>
  <li>포즈가 안정화되면 <code>/pf/stable</code> 토픽 발행</li>
</ul>

---

#### /controller_switcher
<ul>
  <li><code>/state</code> 토픽 구독</li>
  <li>상태에 따라 PP Controller ↔ TLN Controller 전환</li>
</ul>

<br/>

### 🗂️ Code Structure
<hr/>

<pre>
src/
 ├── collision_detector/
 ├── state_machine/
 ├── tln_inference/
 ├── particle_filter/
 ├── controller_switcher/
</pre>

<p>
⚠️ 본 저장소에는 <b>ForzaETH 원본 전체 코드가 아닌</b><br/>
<b>수정 및 추가된 패키지/노드만 포함</b>되어 있다.
</p>

<br/>

### 🧪 Experimental Environment
<hr/>

<table>
  <tr>
    <td><b>Simulation</b></td>
    <td>ForzaETH</td>
  </tr>
  <tr>
    <td><b>Platform</b></td>
    <td>F1TENTH</td>
  </tr>
  <tr>
    <td><b>Main Controller</b></td>
    <td>Pure Pursuit</td>
  </tr>
  <tr>
    <td><b>Recovery Controller</b></td>
    <td>TinyLidarNet</td>
  </tr>
  <tr>
    <td><b>Localization</b></td>
    <td>Particle Filter</td>
  </tr>
</table>

<br/>

### 🚀 Recovery Scenario
<hr/>

<ol>
  <li>Pure Pursuit 기반 정상 주행</li>
  <li>충돌 발생</li>
  <li>Collision Detector 감지</li>
  <li>State Machine이 TLN 후진 모드로 전환</li>
  <li>TinyLidarNet 기반 후진 주행</li>
  <li>Particle Filter 포즈 안정화 확인</li>
  <li>Pure Pursuit 주행 자동 복귀</li>
</ol>

<br/>

### 🧩 Contribution & Significance
<hr/>

<ul>
  <li>학부 수준에서 구현 가능한 <b>실전형 충돌 복구 시스템</b></li>
  <li>알고리즘보다 <b>시스템 통합과 안정성</b>에 초점</li>
  <li>학습 기반 모델과 전통적 제어기의 <b>하이브리드 구조</b></li>
</ul>

<br/>

### 👤 Author
<hr/>

<p>
F1TENTH Project Team<br/>
ForzaETH 기반 Collision Recovery System
</p>
