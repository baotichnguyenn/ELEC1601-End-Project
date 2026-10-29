export function DocsPanel() {
  return (
    <div className="stack small">
      <h2>What runs</h2>
      <p>
        Your sketch is compiled by <b>Clang 22</b> (running as WebAssembly in a worker) to a WebAssembly module, after the same preprocessing the Arduino IDE does (concatenated .ino files, <code>#include &lt;Arduino.h&gt;</code>, generated prototypes). There is no separately written navigation algorithm: if the sketch does not compile or crashes, the run fails.
      </p>
      <p>Hardware calls go to the simulator through a compatibility layer with the AVR core's API (<code>firmware-runtime/</code>):</p>
      <table className="grid">
        <tbody>
          <tr><td>digitalRead on IR receiver pins</td><td>IR model: LOW when the paired IR LED carries a tone() whose detection range reaches a wall in the beam</td></tr>
          <tr><td>tone / noTone</td><td>Timer2 frequency quantisation, one tone at a time</td></tr>
          <tr><td>Servo attach/write/writeMicroseconds/detach</td><td>Pulses every 20 ms drive the wheel model; detach = no pulses = wheels stop</td></tr>
          <tr><td>millis / micros / delay</td><td>Virtual clock; the physics keeps stepping while the firmware waits</td></tr>
          <tr><td>Serial.print/println</td><td>Serial monitor; UART timing (blocks when the 64-byte buffer is full)</td></tr>
          <tr><td>digitalWrite, pinMode</td><td>Pin state; indicator LEDs</td></tr>
          <tr><td>analogRead / analogWrite</td><td>Partial: no analog devices modelled</td></tr>
        </tbody>
      </table>
      <h2>Not supported</h2>
      <p>attachInterrupt, pulseIn, timers and ISRs, direct register access (PORTB, TCCR1A…), EEPROM, Wire, SPI, SoftwareSerial, the String class, Serial input. Using them is a compile error that names the feature.</p>
      <h2>Known differences from the Uno</h2>
      <ul>
        <li><code>int</code> is 32-bit here (16-bit on the Uno) and <code>double</code> is 64-bit (32-bit on the Uno). Code that relies on 16-bit overflow behaves differently.</li>
        <li>The sketch's own computation takes no simulated time; only Arduino calls cost time (configurable cycles per call).</li>
        <li>Integer division by zero crashes the firmware here (reported as a runtime error); the Uno returns garbage.</li>
        <li>2 KB RAM limit is not enforced.</li>
      </ul>
      <h2>Physical model</h2>
      <p>Differential drive with per-wheel servo response curves, motor lag, optional speed noise, 1 ms fixed-step integration, rectangle footprint with collision push-out (frictionless sliding), ray-cast IR beams. Default physical values are <b>assumed</b> or <b>derived from your firmware's calibration constants</b>, not measured. See the Robot tab badges.</p>
      <h2>Workflow</h2>
      <ol>
        <li>Robot tab: enter measured dimensions; record lab measurements.</li>
        <li>Maze tab: set tile, corridor and wall sizes to match the lab maze.</li>
        <li>Firmware tab: upload the current sketch and compile.</li>
        <li>Tests tab: run the baseline suite without changes; save it as the baseline.</li>
        <li>Inspect failures: click a trial to replay, click events to see the robot, sensors and commands at that moment.</li>
        <li>Calibrate tab: change one parameter (or one behaviour in the code), rebuild, rerun the suite, compare with the baseline.</li>
        <li>Save the best configuration as a profile; export results to compare with lab runs.</li>
        <li>When the lab disagrees with the simulator, fix the model (Robot tab) before changing the firmware.</li>
      </ol>
      <p className="muted">Full details: simulator/README.md and simulator/docs/.</p>
    </div>
  );
}
