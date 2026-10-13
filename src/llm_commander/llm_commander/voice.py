"""Offline voice for the web UI: speech-to-text (whisper.cpp) and text-to-speech (Piper). No internet at runtime."""
import collections
import os
import re
import subprocess
import tempfile
import time

# Vocabulary hint for whisper (helps with "tray", "cube", colours). A word list rather than example
# sentences: with unclear audio whisper may echo its hint, and an echoed sentence looks like a real command.
HINT = 'Vocabulary: pick up, place, put, red cube, green cube, blue cube, blue tray, green tray, box, go home.'



class VoiceError(Exception):
    pass


class Transcriber:
    def __init__(self, cfg):
        self.bin = os.path.expanduser(cfg['whisper_bin'])
        self.model = os.path.expanduser(cfg['whisper_model'])
        self.threads = str(cfg.get('threads', 8))

    def available(self):
        return os.access(self.bin, os.X_OK) and os.path.exists(self.model)

    def transcribe(self, wav_bytes):
        """Return (text, seconds) for a WAV recording."""
        if not self.available():
            raise VoiceError(f'whisper.cpp not found ({self.bin}, {self.model})')
        t0 = time.monotonic()
        with tempfile.NamedTemporaryFile(suffix='.wav') as f:
            f.write(wav_bytes)
            f.flush()
            out = subprocess.run([self.bin, '-m', self.model, '-f', f.name, '-t', self.threads, '-l', 'en',
                                  '-nt', '-np', '--prompt', HINT],
                                 capture_output=True, text=True, timeout=60)
        if out.returncode != 0:
            raise VoiceError(f'whisper.cpp failed: {out.stderr.strip()[-300:]}')
        text = re.sub(r'\[[A-Z_ ]+\]|\([^)]*\)', ' ', out.stdout)  # drop [BLANK_AUDIO], (music), ...
        return ' '.join(text.split()), time.monotonic() - t0


class Speaker:
    """Text-to-speech with a local Piper voice; returns WAV bytes (small cache for repeated phrases)."""

    def __init__(self, cfg):
        self.bin = os.path.expanduser(cfg['piper_bin'])
        self.model = os.path.expanduser(cfg['piper_model'])
        self._cache = collections.OrderedDict()

    def available(self):
        return os.access(self.bin, os.X_OK) and os.path.exists(self.model)

    def synth(self, text):
        text = ' '.join(text.split())[:500]
        if text in self._cache:
            self._cache.move_to_end(text)
            return self._cache[text]
        with tempfile.NamedTemporaryFile(suffix='.wav') as f:
            out = subprocess.run([self.bin, '--model', self.model, '--output_file', f.name],
                                 input=text, capture_output=True, text=True, timeout=60)
            if out.returncode != 0:
                raise VoiceError(f'piper failed: {out.stderr.strip()[-300:]}')
            wav = open(f.name, 'rb').read()
        self._cache[text] = wav
        if len(self._cache) > 50:
            self._cache.popitem(last=False)
        return wav
