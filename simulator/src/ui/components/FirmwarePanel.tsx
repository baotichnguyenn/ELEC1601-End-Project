import { useState } from 'react';
import type { BuildResult } from '../../core/firmware/compiler.ts';
import type { Diagnostic, SourceFile } from '../../core/firmware/sketch.ts';
import { pickFiles } from '../storage.ts';
import { CodeEditor } from './CodeEditor.tsx';

interface Props {
  files: SourceFile[];
  setFiles: (f: SourceFile[]) => void;
  active: string;
  setActive: (name: string) => void;
  build: BuildResult | null;
  stale: boolean;
  building: string | null;
  onBuild: () => void;
  onLoadRepoSketch: () => void;
  jump: { file: string; line: number; nonce: number } | null;
}

const ACCEPT = '.ino,.h,.hpp,.cpp,.cc,.c';

export function FirmwarePanel({ files, setFiles, active, setActive, build, stale, building, onBuild, onLoadRepoSketch, jump }: Props) {
  const [showGenerated, setShowGenerated] = useState(false);
  const file = files.find((f) => f.name === active) ?? files[0];
  const diags = build?.diagnostics ?? [];
  const fileDiags = (name: string): Diagnostic[] => diags.filter((d) => d.file === name);

  const upload = async () => {
    const picked = await pickFiles(ACCEPT, true);
    if (!picked.length) return;
    const added = await Promise.all(picked.map(async (f) => ({ name: f.name, content: await f.text() })));
    const replaceAll = files.length > 0 && window.confirm(`Replace the current project with the ${added.length} uploaded file(s)?\nCancel adds them to the current project instead.`);
    const base = replaceAll ? [] : files.filter((f) => !added.some((a) => a.name === f.name));
    setFiles([...base, ...added]);
    setActive(added[0].name);
  };

  const newFile = () => {
    const name = window.prompt('File name (.ino, .h or .cpp)', 'helpers.h');
    if (!name) return;
    if (files.some((f) => f.name === name)) return window.alert('A file with that name exists');
    setFiles([...files, { name, content: name.endsWith('.h') ? '#pragma once\n' : '' }]);
    setActive(name);
  };

  const rename = () => {
    const name = window.prompt('New name', file.name);
    if (!name || name === file.name) return;
    setFiles(files.map((f) => (f === file ? { ...f, name } : f)));
    setActive(name);
  };

  const remove = () => {
    if (!window.confirm(`Delete ${file.name} from the simulator project? (Your files on disk are not touched.)`)) return;
    const rest = files.filter((f) => f !== file);
    setFiles(rest);
    setActive(rest[0]?.name ?? '');
  };

  const errors = diags.filter((d) => d.severity === 'error').length;
  const warnings = diags.filter((d) => d.severity === 'warning').length;

  return (
    <div className="stack">
      <div className="row between">
        <h2 style={{ margin: 0 }}>Project files</h2>
        <div className="row">
          <button className="btn small" onClick={upload} title="Upload .ino, .h and .cpp files">Upload…</button>
          <button className="btn small" onClick={newFile}>New</button>
          <button className="btn small" onClick={onLoadRepoSketch} title="Load ESSENTIAL.ino from the repository folder">Load ESSENTIAL.ino</button>
        </div>
      </div>
      <div className="filelist">
        {files.map((f) => (
          <div key={f.name} className={`file ${f.name === file?.name ? 'active' : ''}`} onClick={() => setActive(f.name)}>
            <span className="name">{f.name}</span>
            {fileDiags(f.name).some((d) => d.severity === 'error') && <span className="badge danger">errors</span>}
            <span className="muted small">{f.content.split('\n').length} lines</span>
          </div>
        ))}
        {!files.length && <div className="empty">No files. Upload your sketch or load ESSENTIAL.ino.</div>}
      </div>
      {file && (
        <>
          <div className="row between">
            <span className="mono">{file.name}</span>
            <div className="row">
              <button className="btn small" onClick={rename}>Rename</button>
              <button className="btn small danger" onClick={remove}>Delete</button>
            </div>
          </div>
          <CodeEditor
            docKey={file.name}
            value={file.content}
            onChange={(content) => setFiles(files.map((f) => (f.name === file.name ? { ...f, content } : f)))}
            diagnostics={fileDiags(file.name)}
            jump={jump && jump.file === file.name ? { line: jump.line, nonce: jump.nonce } : null}
          />
        </>
      )}
      <div className="row between">
        <button className="btn primary" onClick={onBuild} disabled={!!building || !files.length}>
          {building ? 'Compiling…' : 'Compile'}
        </button>
        <span className="small muted">{building ?? (build ? `${build.ok ? 'Built' : 'Failed'} in ${(build.durationMs / 1000).toFixed(1)} s · ${errors} error(s), ${warnings} warning(s)${stale ? ' · sources changed since' : ''}` : 'Not compiled yet')}</span>
      </div>
      {build && (
        <>
          <h2>Arduino API compatibility</h2>
          <table className="grid">
            <thead>
              <tr>
                <th>API</th>
                <th className="num">Uses</th>
                <th>Status</th>
              </tr>
            </thead>
            <tbody>
              {build.apiReport.uses.map((u) => (
                <tr key={u.api} title={u.note}>
                  <td className="mono">{u.api}</td>
                  <td className="num">{u.count}</td>
                  <td>
                    <span className={`badge ${u.status === 'supported' ? 'ok' : u.status === 'partial' ? 'warn' : 'danger'}`}>{u.status}</span> <span className="small muted">{u.note}</span>
                  </td>
                </tr>
              ))}
              {build.apiReport.includes.map((i) => (
                <tr key={i.name + i.file}>
                  <td className="mono">#include &lt;{i.name}&gt;</td>
                  <td className="num">{i.file}:{i.line}</td>
                  <td>
                    <span className={`badge ${i.status === 'supported' ? 'ok' : 'danger'}`}>{i.status}</span>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
          {build.applied.length > 0 && (
            <>
              <h2>Overrides compiled into this build</h2>
              <table className="grid">
                <tbody>
                  {build.applied.map((a) => (
                    <tr key={a.name}>
                      <td className="mono">{a.name}</td>
                      <td className="mono muted">{a.from}</td>
                      <td className="mono">→ {a.to}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </>
          )}
          <details open={showGenerated} onToggle={(e) => setShowGenerated((e.target as HTMLDetailsElement).open)}>
            <summary>Generated translation unit (what Clang compiles) and command</summary>
            <pre className="mono small card" style={{ whiteSpace: 'pre-wrap' }}>{build.command.join(' ')}</pre>
            {showGenerated && <CodeEditor docKey="__generated" value={build.generatedCpp} readOnly />}
          </details>
        </>
      )}
    </div>
  );
}
