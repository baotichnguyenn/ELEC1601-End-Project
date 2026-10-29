import { useEffect, useRef } from 'react';
import { basicSetup } from 'codemirror';
import { cpp } from '@codemirror/lang-cpp';
import { EditorState } from '@codemirror/state';
import { EditorView } from '@codemirror/view';
import { setDiagnostics, type Diagnostic as CmDiagnostic } from '@codemirror/lint';
import type { Diagnostic } from '../../core/firmware/sketch.ts';

interface Props {
  value: string;
  onChange?: (value: string) => void;
  readOnly?: boolean;
  diagnostics?: Diagnostic[];
  /** Scroll to and select this line when `nonce` changes */
  jump?: { line: number; nonce: number } | null;
  docKey: string;
}

export function CodeEditor({ value, onChange, readOnly, diagnostics, jump, docKey }: Props) {
  const host = useRef<HTMLDivElement>(null);
  const view = useRef<EditorView | null>(null);
  const changeRef = useRef(onChange);
  changeRef.current = onChange;

  useEffect(() => {
    const v = new EditorView({
      parent: host.current!,
      state: EditorState.create({
        doc: value,
        extensions: [
          basicSetup,
          cpp(),
          EditorState.readOnly.of(!!readOnly),
          EditorView.updateListener.of((u) => {
            if (u.docChanged) changeRef.current?.(u.state.doc.toString());
          }),
        ],
      }),
    });
    view.current = v;
    return () => v.destroy();
    // A new document (file switch) gets a fresh editor state
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [docKey, readOnly]);

  useEffect(() => {
    const v = view.current;
    if (v && v.state.doc.toString() !== value) v.dispatch({ changes: { from: 0, to: v.state.doc.length, insert: value } });
  }, [value]);

  useEffect(() => {
    const v = view.current;
    if (!v) return;
    const doc = v.state.doc;
    const list: CmDiagnostic[] = (diagnostics ?? [])
      .filter((d) => d.line >= 1 && d.line <= doc.lines)
      .map((d) => {
        const line = doc.line(d.line);
        const from = Math.min(line.to, line.from + Math.max(0, d.column - 1));
        return { from, to: Math.max(from, line.to), severity: d.severity === 'note' ? 'info' : d.severity, message: d.message + (d.hint ? `\n${d.hint}` : '') };
      });
    v.dispatch(setDiagnostics(v.state, list));
  }, [diagnostics, docKey]);

  useEffect(() => {
    const v = view.current;
    if (!v || !jump) return;
    const n = Math.min(Math.max(1, jump.line), v.state.doc.lines);
    const line = v.state.doc.line(n);
    v.dispatch({ selection: { anchor: line.from, head: line.to }, effects: EditorView.scrollIntoView(line.from, { y: 'center' }) });
    v.focus();
  }, [jump]);

  return <div className="editor" ref={host} />;
}
