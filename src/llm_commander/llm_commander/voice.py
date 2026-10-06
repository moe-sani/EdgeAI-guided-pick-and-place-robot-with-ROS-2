"""Offline speech-to-text for voice commands, using a local whisper.cpp build (no internet at runtime)."""
import os
import re
import subprocess
import tempfile
import time

# Words the robot understands; whisper uses this as context, which helps with "tray", "cube", colours...
HINT = ('Pick up the red cube and place it in the blue tray. Put the green cube in the green tray. '
        'Put the blue cube in the box. Go home.')


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
