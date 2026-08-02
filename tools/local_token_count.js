#!/usr/bin/env node
/* Count text tokens locally with VS Code's bundled o200k tokenizer.
 * No repository content leaves the machine.  The optional positional arguments
 * override the worker and vocabulary paths for non-default VS Code installs.
 */

"use strict";

const fs = require("fs");
const { Worker } = require("worker_threads");

const workerPath = process.argv[2]
  || "/usr/share/code/resources/app/extensions/copilot/dist/tikTokenizerWorker.js";
const vocabPath = process.argv[3]
  || "/usr/share/code/resources/app/extensions/copilot/dist/o200k_base.tiktoken";

if (!fs.existsSync(workerPath) || !fs.existsSync(vocabPath)) {
  process.stderr.write("FAIL local tokenizer assets not found; pass WORKER VOCAB paths\n");
  process.exit(2);
}

const input = fs.readFileSync(0, "utf8");
const worker = new Worker(workerPath);
let nextId = 1;
const pending = new Map();

worker.on("message", (message) => {
  const callback = pending.get(message.id);
  if (callback) {
    pending.delete(message.id);
    callback(message);
  }
});
worker.on("error", (error) => {
  process.stderr.write(`FAIL local tokenizer worker: ${error.message}\n`);
  process.exit(2);
});

function call(fn, args) {
  return new Promise((resolve, reject) => {
    const id = nextId++;
    pending.set(id, (message) => {
      if (message.err) reject(new Error(String(message.err)));
      else resolve(message.res);
    });
    worker.postMessage({ id, fn, args });
  });
}

(async () => {
  try {
    // VS Code ships a compact varint vocabulary, so request the worker's binary loader.
    const handle = await call("init", [vocabPath, "o200k_base", true]);
    const tokenIds = await call("encode", [handle, input, undefined]);
    process.stdout.write(`${tokenIds.length}\n`);
    await worker.terminate();
  } catch (error) {
    process.stderr.write(`FAIL local tokenizer: ${error.message}\n`);
    await worker.terminate();
    process.exit(2);
  }
})();
