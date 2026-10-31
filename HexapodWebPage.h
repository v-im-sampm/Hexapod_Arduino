#pragma once

namespace Hexapod
{
const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Hexapod control</title>
<style>
:root{color-scheme:dark;font:16px system-ui,sans-serif;background:#101721;color:#e5edf5}
*{box-sizing:border-box}body{max-width:980px;margin:auto;padding:20px}
h1{margin:0}h2{font-size:1.2rem;margin-top:0}p{line-height:1.5}
.muted{color:#a6b8c9}.warning{border-left:4px solid #ffbd69;padding:12px;background:#30291e}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:16px}
section{padding:18px;border:1px solid #334455;border-radius:12px;background:#182330;margin:16px 0}
.grid section{margin:0}.row{display:flex;gap:10px;flex-wrap:wrap;align-items:end}
button,input,select{font:inherit;border:1px solid #536a80;border-radius:7px;padding:10px;max-width:100%}
button{background:#28425a;color:white;cursor:pointer;min-height:44px}
button:hover{background:#385a77}button:disabled{opacity:.45;cursor:not-allowed}
button.primary{background:#09695e}button.danger{background:#963535}
button.held{background:#09695e;outline:2px solid #7ae9d7}
input,select{background:#0e1925;color:#e5edf5;width:100%}
label{display:grid;gap:5px;flex:1;min-width:100px;margin:6px 0}
form{margin:12px 0}.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:340px;margin:18px auto}
.pad button{touch-action:none;user-select:none;min-height:70px}
.pad .forward{grid-column:2}.pad .left{grid-column:1}.pad .back{grid-column:2}
#status{font-weight:600}pre{white-space:pre-wrap;overflow-wrap:anywhere;max-height:300px;overflow:auto;background:#0e1925;padding:12px}
small{display:block;color:#a6b8c9;line-height:1.4}.wide{width:100%}
</style>
</head>
<body>
<header><h1>Hexapod control</h1><p class="muted">Local control - no Internet required</p></header>
<p class="warning">Open Wi-Fi: anyone nearby can connect and control the robot. Support its body during
setup, keep clear of the legs, and keep a physical power disconnect within reach.
Outputs off releases torque. These buttons are not a hardware emergency stop.</p>
<section aria-label="Connection">
<p id="status" role="status">Connecting...</p>
<p id="details" class="muted"></p>
<div class="row"><button id="enable" class="primary" data-command="web" disabled>Enable driving / home</button>
<button data-command="stop">Stop / hold home</button>
<button class="danger" data-command="off">Outputs off</button></div>
<small>ESP-NOW takes priority and disconnects this AP. After a disconnect, reconnect to the AP and
explicitly enable driving again. Boot and initialization never enable outputs.
Home/startup moves one joint every 250 ms (about 4.5 seconds for all joints); support the body.
Driving waits until ready. Stop cancels pending startup moves and holds only the joints already enabled.</small>
</section>
<div class="grid">
<section><h2>Drive</h2>
<p>Hold a button or W/A/S/D or arrow keys. Release to stop. Forward/backward and turning can be combined.</p>
<div class="pad">
<button class="forward" data-direction="1" data-rotation="0" disabled>Forward</button>
<button class="left" data-direction="0" data-rotation="1" disabled>Left</button>
<button data-command="stop">Stop</button>
<button data-direction="0" data-rotation="-1" disabled>Right</button>
<button class="back" data-direction="-1" data-rotation="0" disabled>Reverse</button>
</div>
<small>A 500 ms missing-heartbeat watchdog returns toward home. A hidden tab, lost focus, or connection
error ends this driving session; it does not resume automatically.</small>
</section>
<section><h2>Servo sequence</h2>
<p>Outputs must be off to start. Each joint moves 90 / 85 / 95 / 90 degrees, then pauses after each leg.</p>
<div class="row"><button data-command="testservos">Start test</button>
<button id="next" data-command="next" disabled>Next leg / finish</button>
<button id="retest" data-command="retest" disabled>Retest leg</button>
<button data-command="stop">Cancel / hold</button></div>
<p id="sequence" class="muted">No servo test running.</p>
<small>Leg pauses release torque. Tests and calibration poses are not browser driving:
they retain their normal sequence/holding behavior if the page disconnects. Use serial or the physical
power disconnect if ESP-NOW removes the AP.</small>
</section>
<section><h2>Joint positioning</h2>
<div class="row"><label>Leg<select id="leg"><option>rf</option><option>rm</option><option>rr</option>
<option>lr</option><option>lm</option><option>lf</option></select></label>
<label>Joint<select id="joint"><option>coxa</option><option>femur</option><option>tibia</option></select></label></div>
<form id="angle-form"><div class="row"><label>Angle (degrees)<input id="angle" type="number" min="0" max="180" step="any" value="90" required></label>
<button>Set angle</button></div></form>
<form id="pulse-form"><div class="row"><label>Pulse (microseconds)<input id="pulse" type="number" min="500" max="2500" step="1" value="1500" required></label>
<button>Set pulse</button></div></form>
<div class="row"><button id="center-leg">Center this leg</button><button data-command="center all">Center all</button></div>
<small>Requires off or pose mode. Moves are queued one joint every 250 ms; Stop cancels pending moves.
Check clearance before moving. Previously positioned joints keep holding.</small>
</section>
<section><h2>Calibration</h2>
<p>Uses the leg and joint selected under Joint positioning. Outputs must be off.</p>
<button data-command="show">Read current settings (off)</button>
<form id="cal-form">
<div class="row"><label>Minimum us<input id="min-us" type="number" min="500" max="2499" required></label>
<label>Maximum us<input id="max-us" type="number" min="501" max="2500" required></label></div>
<div class="row"><label>Trim degrees<input id="trim" type="number" min="-90" max="90" step="any" required></label>
<label>Reverse<select id="reverse"><option value="0">No</option><option value="1">Yes</option></select></label></div>
<button>Apply calibration to RAM</button></form>
<small>Read current settings first or enter every value. Only Save writes flash. There is no automatic calibration.</small>
</section>
</div>
<section><h2>Configuration and console</h2>
<form id="param-form"><div class="row"><label>Parameter name<input id="param" placeholder="step_length" pattern="[a-zA-Z0-9_]+" required></label>
<label>Value<input id="value" type="number" step="any" required></label><button>Apply parameter</button></div></form>
<div class="row"><button data-command="params">Parameter ranges</button><button data-command="save">Save to flash</button>
<button data-command="load">Load saved settings</button><button id="defaults">Factory defaults in RAM</button>
<button data-command="init">Initialize drivers / radio</button><button data-command="help">Help</button></div>
<form id="console-form"><div class="row"><label>Console command<input id="command" maxlength="159"
placeholder="status, origin rf 103 -60, joints rf 90 90 90..." autocomplete="off"></label><button>Send</button></div></form>
<small>Same validation and off-mode requirements as serial. Enter on an empty command advances a paused test.
Changing radio_channel and running init can disconnect Wi-Fi; reconnect afterward.</small>
<pre id="log" role="log" aria-live="polite">Waiting for controller status.</pre>
</section>
<script>
'use strict';
const $ = id => document.getElementById(id);
let token = '', generation = 0, driveGeneration = 0, enabled = false, online = false;
let commandBusy = false, driveBusy = false, statusBusy = false, version = 0;
const held = new Map(), calibrations = new Map();
const driveButtons = [...document.querySelectorAll('[data-direction]')];
function log(text) { $('log').textContent = (text + '\n\n' + $('log').textContent).slice(0, 6000); }
function clearHeld() {
  held.clear();
  driveButtons.forEach(button => button.classList.remove('held'));
}
function localStop() {
  enabled = false;
  clearHeld();
  driveButtons.forEach(button => button.disabled = true);
}
function disconnected(error) {
  localStop();
  online = false;
  $('enable').disabled = true;
  $('status').textContent = 'Disconnected - motion will stop; reconnect and enable again.';
  log(error.message || String(error));
}
async function request(path, payload, requestGeneration = generation, keepalive = false) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 2500);
  try {
    const options = {cache:'no-store', signal:controller.signal, keepalive};
    if (payload !== undefined) {
      options.method = 'POST';
      options.headers = {'Content-Type':'text/plain', 'X-Hexapod-Token':token};
      options.body = requestGeneration + '\n' + payload;
    }
    const response = await fetch(path, options);
    const text = await response.text();
    const next = response.headers.get('X-Hexapod-Generation');
    return {ok:response.ok, text, generation:next === null ? requestGeneration : Number(next)};
  } finally { clearTimeout(timer); }
}
function fillCalibration() {
  const values = calibrations.get($('leg').value + ' ' + $('joint').value) || ['', '', '', '0'];
  ['min-us','max-us','trim','reverse'].forEach((id, index) => $(id).value = values[index]);
}
async function command(text) {
  const ownVersion = ++version;
  localStop();
  commandBusy = true;
  try {
    if (!online) throw new Error('Not connected. Reconnect to the Hexapod AP and wait for status.');
    const result = await request('/api/command', text);
    if (ownVersion !== version) return;
    generation = result.generation;
    log('> ' + text + '\n' + result.text);
    if (result.ok && text.toLowerCase() === 'show') {
      calibrations.clear();
      for (const line of result.text.split('\n')) {
        const parts = line.trim().split(/\s+/);
        if (parts[0] === 'cal' && parts.length === 7)
          calibrations.set(parts[1] + ' ' + parts[2], parts.slice(3));
      }
      fillCalibration();
    }
    if (result.ok && text.toLowerCase() === 'web' && !document.hidden) {
      enabled = true;
      driveGeneration = generation;
      driveButtons.forEach(button => button.disabled = false);
    }
  } catch (error) {
    if (ownVersion === version) disconnected(error);
  } finally {
    if (ownVersion === version) {
      commandBusy = false;
      if (enabled) heartbeat();
      else pollStatus();
    }
  }
}
function updateStatus(text) {
  const state = JSON.parse(text);
  token = state.token;
  generation = state.generation;
  online = true;
  if (enabled && (state.mode !== 'web' || generation !== driveGeneration)) localStop();
  $('status').textContent = 'Connected | Mode: ' + state.mode + (enabled ? ' | Driving enabled' : '');
  $('details').textContent = (state.drivers ? 'Drivers ready' : 'Drivers need init') +
    ' | ' + (state.dirty ? 'Unsaved changes' : 'Settings saved/default') +
    (state.radioReady ? ' | Listening for ESP-NOW' : ' | ESP-NOW needs init') +
    (state.posePending ? ' | Moving joints in sequence - please wait' : '');
  $('enable').disabled = !state.drivers;
  $('next').disabled = $('retest').disabled = state.mode !== 'testservos-paused';
  $('sequence').textContent = state.mode.startsWith('testservos') ?
    state.leg.toUpperCase() + ' / ' + state.joint + ' / ' +
    (state.mode === 'testservos-paused' ? 'Paused - Next or Retest' : 'Step ' + (state.step + 1)) :
    'No servo test running.';
}
async function heartbeat() {
  if (!enabled || driveBusy || commandBusy || document.hidden) return;
  let direction = 0, rotation = 0;
  for (const value of held.values()) { direction += value[0]; rotation += value[1]; }
  direction = Math.max(-1, Math.min(1, direction));
  rotation = Math.max(-1, Math.min(1, rotation));
  const ownVersion = version;
  driveBusy = true;
  try {
    const result = await request('/api/drive', direction + ' ' + rotation, driveGeneration);
    if (ownVersion !== version) return;
    if (!result.ok || result.generation !== driveGeneration) {
      localStop();
      log(result.text || 'Control source changed. Explicitly enable driving again.');
    }
    else updateStatus(result.text);
  } catch (error) { if (ownVersion === version) disconnected(error); }
  finally { driveBusy = false; }
}
async function pollStatus() {
  // The HTTP server is serial: driving replies carry status so polls cannot delay heartbeats.
  if (enabled || driveBusy || statusBusy || commandBusy || document.hidden) return;
  statusBusy = true;
  const ownVersion = version;
  try {
    const result = await request('/api/status');
    if (ownVersion !== version) return;
    if (!result.ok) throw new Error(result.text);
    updateStatus(result.text);
  } catch (error) {
    if (ownVersion === version && (online || $('status').textContent === 'Connecting...'))
      disconnected(error);
  }
  finally { statusBusy = false; }
}
document.querySelectorAll('[data-command]').forEach(button =>
  button.addEventListener('click', () => command(button.dataset.command)));
driveButtons.forEach(button => {
  button.addEventListener('pointerdown', event => {
    if (!enabled || event.button !== 0) return;
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    held.set('p' + event.pointerId, [Number(button.dataset.direction), Number(button.dataset.rotation)]);
    button.classList.add('held');
    heartbeat();
  });
  const release = event => {
    held.delete('p' + event.pointerId);
    button.classList.remove('held');
    heartbeat();
  };
  ['pointerup','pointercancel','lostpointercapture'].forEach(name => button.addEventListener(name, release));
});
const keys = {w:[1,0],ArrowUp:[1,0],s:[-1,0],ArrowDown:[-1,0],
              a:[0,1],ArrowLeft:[0,1],d:[0,-1],ArrowRight:[0,-1]};
document.addEventListener('keydown', event => {
  if (!enabled || /INPUT|SELECT|TEXTAREA/.test(event.target.tagName)) return;
  const value = keys[event.key] || keys[event.key.toLowerCase()];
  if (!value) return;
  event.preventDefault();
  held.set('k' + event.code, value);
  heartbeat();
});
document.addEventListener('keyup', event => { held.delete('k' + event.code); heartbeat(); });
function leavePage() {
  const wasEnabled = enabled;
  ++version;
  commandBusy = false;
  localStop();
  if (wasEnabled)
    request('/api/command', 'stop', driveGeneration, true).then(result => {
      if (!result.ok) log(result.text);
    }).catch(error => log('Stop not delivered: ' + error.message + '; watchdog will stop motion.'));
}
window.addEventListener('blur', leavePage);
window.addEventListener('pagehide', leavePage);
document.addEventListener('visibilitychange', () => { if (document.hidden) leavePage(); else pollStatus(); });
function form(id, build) {
  $(id).addEventListener('submit', event => { event.preventDefault(); command(build()); });
}
const joint = () => $('leg').value + ' ' + $('joint').value;
form('angle-form', () => 'servo ' + joint() + ' ' + $('angle').value);
form('pulse-form', () => 'pulse ' + joint() + ' ' + $('pulse').value);
form('cal-form', () => 'cal ' + joint() + ' ' +
  ['min-us','max-us','trim','reverse'].map(id => $(id).value).join(' '));
form('param-form', () => 'set ' + $('param').value + ' ' + $('value').value);
form('console-form', () => $('command').value.trim() || 'next');
$('leg').addEventListener('change', fillCalibration);
$('joint').addEventListener('change', fillCalibration);
$('center-leg').addEventListener('click', () => command('center ' + $('leg').value));
$('defaults').addEventListener('click', () => {
  if (confirm('Replace RAM settings with factory defaults? Flash is unchanged until Save.')) command('defaults');
});
setInterval(heartbeat, 50);
setInterval(pollStatus, 500);
pollStatus();
</script>
</body>
</html>)HTML";
}
