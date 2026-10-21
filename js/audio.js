// Synthesised handpiece sound: motor hum + cutter chatter that rises while
// cutting through wool.
export class ClipperAudio {
  constructor() { this.ctx = null; this.volume = 0.5; }

  _init() {
    if (this.ctx) return;
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return;
    const ctx = this.ctx = new AC();
    this.master = ctx.createGain(); this.master.gain.value = 0; this.master.connect(ctx.destination);
    // motor
    const o1 = ctx.createOscillator(); o1.type = 'sawtooth'; o1.frequency.value = 96;
    const o2 = ctx.createOscillator(); o2.type = 'square'; o2.frequency.value = 192.5;
    const lp = ctx.createBiquadFilter(); lp.type = 'lowpass'; lp.frequency.value = 900;
    const g1 = ctx.createGain(); g1.gain.value = 0.18;
    const g2 = ctx.createGain(); g2.gain.value = 0.05;
    o1.connect(g1).connect(lp); o2.connect(g2).connect(lp); lp.connect(this.master);
    // cutter chatter (filtered noise, amplitude-modulated at the stroke rate)
    const buf = ctx.createBuffer(1, ctx.sampleRate, ctx.sampleRate);
    const d = buf.getChannelData(0);
    for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1;
    const noise = ctx.createBufferSource(); noise.buffer = buf; noise.loop = true;
    const bp = ctx.createBiquadFilter(); bp.type = 'bandpass'; bp.frequency.value = 2600; bp.Q.value = 0.8;
    this.chatter = ctx.createGain(); this.chatter.gain.value = 0;
    const am = ctx.createOscillator(); am.frequency.value = 50;
    const amGain = ctx.createGain(); amGain.gain.value = 0.5;
    am.connect(amGain).connect(this.chatter.gain);
    noise.connect(bp).connect(this.chatter).connect(this.master);
    o1.start(); o2.start(); noise.start(); am.start();
    this.motor = [o1, o2];
  }

  /** running: motor on; load: 0..1 wool being cut */
  update(running, load) {
    if (running) this._init();
    if (!this.ctx) return;
    if (this.ctx.state === 'suspended' && running) this.ctx.resume();
    const t = this.ctx.currentTime;
    this.master.gain.setTargetAtTime(running ? 0.25 * this.volume : 0, t, 0.05);
    this.chatter.gain.setTargetAtTime(running ? 0.15 + 0.85 * load : 0, t, 0.04);
    const slow = 1 - 0.08 * load; // motor bogs down slightly under load
    this.motor[0].frequency.setTargetAtTime(96 * slow, t, 0.08);
    this.motor[1].frequency.setTargetAtTime(192.5 * slow, t, 0.08);
  }
}
