// Find a REAL bash, and run one of this repo's shell scripts through it.
//
// `"quickstart": "bash scripts/quickstart.sh"` looks fine and fails on Windows:
// npm/pnpm spawn scripts through cmd.exe, where `bash` resolves to
// C:\Windows\System32\bash.exe — the WSL launcher — which cannot see the Git
// Bash toolchain and dies with `execvpe(/bin/bash) failed`. So we resolve Git
// Bash ourselves and never trust a bare `bash` on win32.
//
// Shared by every scripts/*.mjs launcher (quickstart, clear), which is the only
// reason it is a module rather than inline.
import { spawn, execFileSync } from "node:child_process";
import { existsSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));

export function findBash() {
  if (process.platform !== "win32") return "bash";
  if (process.env.GIT_BASH && existsSync(process.env.GIT_BASH)) return process.env.GIT_BASH;

  const candidates = [
    join(process.env.ProgramFiles ?? "C:\\Program Files", "Git", "bin", "bash.exe"),
    join(process.env["ProgramFiles(x86)"] ?? "C:\\Program Files (x86)", "Git", "bin", "bash.exe"),
    join(process.env.LOCALAPPDATA ?? "", "Programs", "Git", "bin", "bash.exe"),
  ];
  // Wherever git itself lives, bash.exe is two levels up in bin/ (…/Git/cmd/git.exe).
  try {
    const gitExe = execFileSync("where", ["git"], { encoding: "utf8" }).split(/\r?\n/)[0].trim();
    if (gitExe) candidates.push(resolve(dirname(gitExe), "..", "bin", "bash.exe"));
  } catch {
    /* git not on PATH — the explicit candidates may still hit */
  }
  const found = candidates.find((p) => p && existsSync(p));
  if (found) return found;

  console.error(
    "Git Bash not found. Run it directly from a Git Bash shell:\n" +
      "  bash scripts/<script>.sh\n" +
      "or set GIT_BASH=C:\\path\\to\\Git\\bin\\bash.exe",
  );
  process.exit(1);
}

/** Run scripts/<name> with the caller's argv, mirroring how it exits. */
export function runScript(name, args = process.argv.slice(2)) {
  const child = spawn(findBash(), [join(here, name), ...args], {
    stdio: "inherit",
    windowsHide: false,
  });
  // Ctrl-C reaches the child through the console's process group; just mirror how it ended.
  child.on("exit", (code, signal) => process.exit(signal ? 1 : (code ?? 0)));
}
