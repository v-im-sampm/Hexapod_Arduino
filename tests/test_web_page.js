'use strict';

const assert = require('node:assert/strict');
const {readFileSync} = require('node:fs');
const {join} = require('node:path');
const {test} = require('node:test');
const vm = require('node:vm');

const source = readFileSync(join(__dirname, '..', 'HexapodWebPage.h'), 'utf8');
const script = source.match(/<script>([\s\S]*?)<\/script>/)[1];

function element() {
  return {
    textContent: '', disabled: false, value: '', dataset: {},
    classList: {add() {}, remove() {}},
    listeners: new Map(),
    addEventListener(name, callback) { this.listeners.set(name, callback); }
  };
}

function createPage() {
  const elements = new Map([...source.matchAll(/id="([^"]+)"/g)]
    .map(match => [match[1], element()]));
  const buttons = Array.from({length: 4}, element);
  const requests = [], intervals = new Map(), timeouts = new Map();
  const document = {
    hidden: false,
    getElementById: id => elements.get(id),
    querySelectorAll: selector => selector === '[data-direction]' ? buttons : [],
    addEventListener() {}
  };
  const window = element();
  const context = vm.createContext({
    document, window, AbortController,
    setInterval(callback, ms) { intervals.set(ms, callback); },
    setTimeout(callback) { const id = {}; timeouts.set(id, callback); return id; },
    clearTimeout(id) { timeouts.delete(id); },
    fetch(path, options) {
      return new Promise((resolve, reject) => requests.push({path, options, resolve, reject}));
    }
  });
  vm.runInContext(script, context);
  const flush = async () => { for (let i = 0; i < 10; ++i) await Promise.resolve(); };
  async function reply(request, body, generation = 2, ok = true) {
    request.resolve({
      ok,
      text: async () => typeof body === 'string' ? body : JSON.stringify(body),
      headers: {get: () => String(generation)}
    });
    await flush();
  }
  return {
    elements, buttons, requests, intervals, document, window, flush, reply,
    evaluate: code => vm.runInContext(code, context)
  };
}

function status(overrides = {}) {
  return {
    token: 'test-token', generation: 2, mode: 'web', drivers: true,
    radioReady: true, dirty: false, posePending: true, leg: 'rf', joint: 'coxa', step: 0,
    direction: 0, rotation: 0, ...overrides
  };
}

async function enable(page) {
  await page.reply(page.requests.shift(), status({generation: 1, mode: 'off'}), 1);
  const pending = page.evaluate("command('web')");
  const command = page.requests.shift();
  assert.equal(command.path, '/api/command');
  assert.equal(command.options.body, '1\nweb');
  await page.reply(command, 'Web driving selected.');
  await pending;
}

test('enable sends a neutral heartbeat immediately, before any status poll', async () => {
  const page = createPage();
  await enable(page);
  assert.equal(page.requests.length, 1);
  assert.equal(page.requests[0].path, '/api/drive');
  assert.equal(page.requests[0].options.body, '2\n0 0');
  assert.equal(page.requests[0].options.headers['X-Hexapod-Token'], 'test-token');
});

test('heartbeats carry status throughout startup without competing status requests', async () => {
  const page = createPage();
  await enable(page);
  for (let elapsed = 0; elapsed <= 5000; elapsed += 50) {
    assert.equal(page.requests.length, 1);
    const drive = page.requests.shift();
    assert.equal(drive.path, '/api/drive');
    page.intervals.get(500)();
    page.intervals.get(50)();
    assert.equal(page.requests.length, 0, 'only one heartbeat may be in flight');
    await page.reply(drive, status({posePending: elapsed < 4500}));
    assert.match(page.elements.get('status').textContent, /Driving enabled/);
    assert.equal(page.elements.get('details').textContent.includes('Moving joints'), elapsed < 4500);
    page.intervals.get(500)();
    assert.equal(page.requests.length, 0, 'status polling must not queue ahead of driving');
    page.intervals.get(50)();
  }
});

test('a delayed drive reply does not end the session or queue status polls', async () => {
  const page = createPage();
  await enable(page);
  const drive = page.requests.shift();
  for (let elapsed = 0; elapsed <= 1500; elapsed += 50) {
    page.intervals.get(50)();
    page.intervals.get(500)();
    assert.equal(page.requests.length, 0);
    assert.equal(page.evaluate('enabled'), true);
  }
  await page.reply(drive, status({posePending: false}));
  assert.equal(page.evaluate('enabled'), true);
  assert.equal(page.elements.get('details').textContent.includes('Moving joints'), false);
  page.intervals.get(50)();
  assert.equal(page.requests.shift().path, '/api/drive');
});

test('drive input and release still use the current session', async () => {
  const page = createPage();
  await enable(page);
  await page.reply(page.requests.shift(), status());
  page.evaluate("held.set('test', [1, -1])");
  page.intervals.get(50)();
  const drive = page.requests.shift();
  assert.equal(drive.options.body, '2\n1 -1');
  await page.reply(drive, status({direction: 1, rotation: -1}));
  page.evaluate('clearHeld()');
  page.intervals.get(50)();
  assert.equal(page.requests.shift().options.body, '2\n0 0');
});

test('a rejected session stops heartbeats and polling never re-enables driving', async () => {
  const page = createPage();
  await enable(page);
  await page.reply(page.requests.shift(), 'ERROR: control source changed', 3, false);
  assert.equal(page.evaluate('enabled'), false);
  assert.match(page.elements.get('log').textContent, /control source changed/);
  page.intervals.get(50)();
  assert.equal(page.requests.length, 0);
  page.intervals.get(500)();
  assert.equal(page.requests[0].path, '/api/status');
  await page.reply(page.requests.shift(), status({generation: 3, mode: 'home'}), 3);
  page.intervals.get(50)();
  assert.equal(page.requests.length, 0);
  assert.ok(page.buttons.every(button => button.disabled));
});

test('connection loss attempts to stop and warns if delivery fails', async () => {
  const page = createPage();
  await enable(page);
  page.requests.shift().reject(new Error('Connection lost'));
  await page.flush();
  assert.equal(page.evaluate('enabled'), false);
  assert.match(page.elements.get('status').textContent, /Disconnected/);
  assert.match(page.elements.get('status').textContent, /motion may continue/);
  const stop = page.requests.shift();
  assert.equal(stop.options.body, '2\nstop');
  assert.equal(stop.options.keepalive, true);
  stop.reject(new Error('Still disconnected'));
  await page.flush();
  assert.match(page.elements.get('log').textContent, /Stop not delivered: Still disconnected; motion may continue/);
  assert.match(page.elements.get('log').textContent, /Serial stop\/off/);
  page.intervals.get(50)();
  assert.equal(page.requests.length, 0);
});

test('console commands explicitly stop driving before ending heartbeat updates', async () => {
  const page = createPage();
  await enable(page);
  await page.reply(page.requests.shift(), status());
  const pending = page.evaluate("command('help')");
  const stop = page.requests.shift();
  assert.equal(stop.options.body, '2\nstop');
  assert.equal(page.evaluate('enabled'), false);
  await page.reply(stop, 'Motion stopped.', 3);
  const command = page.requests.shift();
  assert.equal(command.options.body, '3\nhelp');
  await page.reply(command, 'Commands', 3);
  await pending;
  assert.equal(page.requests.shift().path, '/api/status');
});

test('failed stop prevents the following console command and warns the user', async () => {
  const page = createPage();
  await enable(page);
  await page.reply(page.requests.shift(), status());
  const pending = page.evaluate("command('help')");
  await page.reply(page.requests.shift(), 'ERROR: controller busy', 2, false);
  await pending;
  assert.equal(page.requests.length, 0);
  assert.equal(page.evaluate('enabled'), false);
  assert.equal(page.evaluate('commandBusy'), false);
  assert.match(page.elements.get('status').textContent, /motion may continue/);
  assert.match(page.elements.get('log').textContent, /controller busy/);
});

test('stop and outputs off are sent directly without another stop request', async () => {
  for (const text of ['stop', 'off', '!']) {
    const page = createPage();
    await enable(page);
    await page.reply(page.requests.shift(), status());
    const pending = page.evaluate(`command('${text}')`);
    const command = page.requests.shift();
    assert.equal(command.options.body, '2\n' + text);
    await page.reply(command, 'Stopped.', 3);
    await pending;
    assert.equal(page.requests.shift().path, '/api/status');
    assert.equal(page.requests.length, 0);
  }
});

test('leaving the page stops the session and ignores an in-flight heartbeat reply', async () => {
  const page = createPage();
  await enable(page);
  const drive = page.requests.shift();
  page.document.hidden = true;
  page.window.listeners.get('blur')();
  const stop = page.requests.shift();
  assert.equal(stop.path, '/api/command');
  assert.equal(stop.options.body, '2\nstop');
  assert.equal(stop.options.keepalive, true);
  await page.reply(drive, status());
  await page.reply(stop, 'Motion stopped.', 3);
  page.intervals.get(50)();
  assert.equal(page.requests.length, 0);
  assert.equal(page.evaluate('enabled'), false);
});

test('a rejected leave-page stop warns instead of promising a watchdog stop', async () => {
  const page = createPage();
  await enable(page);
  await page.reply(page.requests.shift(), status());
  page.window.listeners.get('blur')();
  await page.reply(page.requests.shift(), 'ERROR: controller busy', 2, false);
  assert.match(page.elements.get('log').textContent, /Stop not delivered: ERROR: controller busy; motion may continue/);
  assert.match(page.elements.get('log').textContent, /physical power disconnect/);
  assert.equal(page.evaluate('enabled'), false);
});
