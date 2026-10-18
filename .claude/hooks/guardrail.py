#!/usr/bin/env python3
"""Vibe コーディング用ガードレール (Claude Code PreToolUse フック)

受講生のワークフォルダ (workshop/<term_no>/<fname-lname>/) の外への
ファイル変更・削除をブロックする。

- 発動条件: セッションの起動ディレクトリが workshop/ 配下のときだけ有効。
  リポジトリ直下などで起動した場合(講師の教材メンテ等)は何もしない。
- 無効化: 環境変数 DRONESCHOOL_GUARDRAIL=off で無効化できる(講師用)。
- ブロック時: exit 2 + stderr のメッセージがエージェントに返る。
"""
import json
import os
import re
import shlex
import sys
import tempfile

# パスの引数をすべてワークフォルダ内に限定するコマンド(削除・移動系)
DESTRUCTIVE_CMDS = {"rm", "rmdir", "unlink", "shred", "truncate", "mv"}
# コピー系: 書き込み先(最後のパス引数)だけワークフォルダ内に限定
# (教材 webapp-blueos/ からワークフォルダへのコピーは許可するため)
COPY_CMDS = {"cp", "rsync"}
# コマンド名の前に付いていても読み飛ばすラッパー
WRAPPERS = {
    "command", "builtin", "exec", "nohup", "time", "nice", "xargs", "env",
}

# ワークフォルダ内であっても常にブロックするコマンド
BLOCKED_PATTERNS = [
    (r"\bsudo\b", "sudo"),
    (r"\bgit\s+(-\S+\s+)*clean\b", "git clean"),
    (r"\bgit\s+(-\S+\s+)*reset\b[^;|&]*--hard", "git reset --hard"),
    (r"\bgit\s+(-\S+\s+)*checkout\b", "git checkout"),
    (r"\bgit\s+(-\S+\s+)*restore\b", "git restore"),
    (r"\bgit\s+(-\S+\s+)*rebase\b", "git rebase"),
    (r"\bgit\s+(-\S+\s+)*push\b[^;|&]*(\s--force\b|\s-f\b|\s\+\S)",
     "git push --force"),
    (r"\bgit\s+(-\S+\s+)*stash\s+(drop|clear)\b", "git stash drop/clear"),
    (r"\bmkfs\b|\bdd\s+if=", "ディスク直接操作"),
]


def deny(message):
    print(f"[ガードレール] {message}", file=sys.stderr)
    sys.exit(2)


def find_project_dir(cwd):
    project = os.environ.get("CLAUDE_PROJECT_DIR")
    if project:
        return os.path.realpath(project)
    d = os.path.realpath(cwd)
    while d != os.path.dirname(d):
        if os.path.isdir(os.path.join(d, ".git")):
            return d
        d = os.path.dirname(d)
    return None


def work_folder(project, cwd):
    """cwd が workshop/ 配下なら、境界となるワークフォルダを返す。

    workshop/<term_no>/<fname-lname>/ の深さ(2階層)までを境界にする。
    workshop/ 配下でなければ None(ガードレール休止)。
    """
    workshop = os.path.join(project, "workshop")
    real_cwd = os.path.realpath(cwd)
    if not (real_cwd == workshop or real_cwd.startswith(workshop + os.sep)):
        return None
    rel = os.path.relpath(real_cwd, workshop)
    parts = [p for p in rel.split(os.sep) if p not in (".", "")]
    return os.path.join(workshop, *parts[:2]) if parts else workshop


def inside(path, root):
    return path == root or path.startswith(root + os.sep)


def allowed_zones(boundary):
    zones = [boundary, "/tmp", tempfile.gettempdir()]
    if os.environ.get("TMPDIR"):
        zones.append(os.path.realpath(os.environ["TMPDIR"]))
    return zones


def path_allowed(token, cwd, zones):
    """パスらしいトークンが許可ゾーン内に収まるか判定する。"""
    if "$" in token:
        return False  # 変数入りは解決できないので不許可
    expanded = os.path.expanduser(token)
    if not os.path.isabs(expanded):
        # 相対パスは起動ディレクトリ基準で解決(.. を含まなければ許可)
        if ".." not in expanded.split("/"):
            return True
        expanded = os.path.join(cwd, expanded)
    # glob はディレクトリ部分だけで判定する
    expanded = re.sub(r"[*?\[].*$", "", expanded)
    resolved = os.path.realpath(expanded)
    return any(inside(resolved, z) for z in zones)


def is_pathlike(token):
    return token.startswith(("/", "~", "./", "../")) or "/" in token or token == ".."


def check_bash(command, cwd, boundary):
    for pattern, name in BLOCKED_PATTERNS:
        if re.search(pattern, command):
            deny(
                f"危険なコマンド({name})はブロックされています。"
                "必要な場合は実行せず、コマンドを提示してユーザー自身に実行してもらってください。"
            )

    try:
        lex = shlex.shlex(command, posix=True, punctuation_chars=True)
        lex.whitespace_split = True
        tokens = list(lex)
    except ValueError:
        deny("コマンドを解析できないためブロックしました。"
             "より単純な形で実行してください。")

    zones = allowed_zones(boundary)

    # ; && || | & でコマンド列に分割して個別にチェック
    segments, seg = [], []
    for tok in tokens:
        if tok and all(c in ";|&()" for c in tok):
            if seg:
                segments.append(seg)
            seg = []
        else:
            seg.append(tok)
    if seg:
        segments.append(seg)

    for seg in segments:
        # 環境変数代入とラッパーを読み飛ばして実コマンドを特定
        i = 0
        while i < len(seg) and (
            re.match(r"^[A-Za-z_][A-Za-z0-9_]*=", seg[i])
            or os.path.basename(seg[i]) in WRAPPERS
        ):
            i += 1
        if i >= len(seg):
            continue
        cmd = os.path.basename(seg[i])
        args = seg[i + 1:]
        path_args = [
            a for a in args if not a.startswith("-") and is_pathlike(a)
        ]

        find_deletes = ("-delete", "-exec", "-execdir")
        targets = []
        if cmd in DESTRUCTIVE_CMDS:
            targets = path_args
        elif cmd in COPY_CMDS:
            targets = path_args[-1:]  # 書き込み先のみ
        elif cmd == "find" and any(a in find_deletes for a in args):
            targets = path_args

        for t in targets:
            if not path_allowed(t, cwd, zones):
                deny(
                    f"ワークフォルダ({boundary})の外を対象とした"
                    f" {cmd} はブロックされました(対象: {t})。"
                    "教材(webapp-blueos/ など)や他の受講生のフォルダは"
                    "変更・削除できません。"
                )


def check_file_write(tool_input, boundary):
    path = tool_input.get("file_path") or tool_input.get("notebook_path")
    if not path:
        return
    resolved = os.path.realpath(os.path.expanduser(path))
    if not any(inside(resolved, z) for z in allowed_zones(boundary)):
        deny(
            f"ワークフォルダ({boundary})の外への書き込みはブロックされました"
            f"(対象: {path})。作業は自分のワークフォルダ配下だけで行ってください。"
        )


def main():
    switch = os.environ.get("DRONESCHOOL_GUARDRAIL", "").lower()
    if switch in ("off", "0", "false"):
        sys.exit(0)
    try:
        data = json.load(sys.stdin)
    except Exception:
        sys.exit(0)

    cwd = data.get("cwd") or os.getcwd()
    project = find_project_dir(cwd)
    if not project:
        sys.exit(0)
    boundary = work_folder(project, cwd)
    if boundary is None:
        sys.exit(0)  # workshop/ 外で起動(講師の教材メンテ等)は対象外

    tool = data.get("tool_name", "")
    tool_input = data.get("tool_input", {}) or {}
    if tool == "Bash":
        check_bash(tool_input.get("command", ""), cwd, boundary)
    elif tool in ("Edit", "Write", "MultiEdit", "NotebookEdit"):
        check_file_write(tool_input, boundary)
    sys.exit(0)


if __name__ == "__main__":
    main()
