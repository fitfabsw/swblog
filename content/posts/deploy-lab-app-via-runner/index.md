---
title: "把「依賴本機資料」的內部工具自動部署：lab-mac-mini + GitHub Runner + it-server 反代"
date: 2026-06-25T15:00:00+08:00
author: "Kevin Lee"
description: "以 atlas-log-wizard 為例，提供一套可逐步 follow 的 SOP：當工具依賴某台機器的本機資料、不適合搬伺服器時，如何讓它跑在原機並做到 push 即自動部署、反代到統一網址。"
summary: "一套可照抄的步驟：app 跑在資料所在的 lab-mac-mini、裝 GitHub self-hosted runner 做 push 自動部署、it-server nginx 反代給統一網址。文末附踩雷註記。"
draft: false
tags: ["部署", "CI/CD", "GitHub Actions", "self-hosted runner", "nginx", "macOS", "Atlas2"]
categories: ["教學"]
series: ["內部部署實戰"]
ShowToc: true
TocOpen: true
---

## 適用情境

大多數內部工具部署在 **it-server**（push → 自動上線）。但有一類工具**依賴某台特定機器上的大量本機資料**，不適合搬到伺服器。

本文以 [atlas-log-wizard](http://10.35.40.81/atlas/) 為例 —— 它要讀的測試 log 有 2.5 GB，放在 **lab-mac-mini**。我們讓 app 直接跑在 lab-mac-mini（就近讀資料），再補上「push 自動部署」與「對外統一網址」。

> ⚠️ 這套是**特例**，不是預設。新工具請優先直接部署 it-server；只有當它被資料或硬體綁死在某台機器時，才用這套。（理由見文末。）

## 架構

```
瀏覽器 → http://10.35.40.81/atlas/      (it-server nginx 反代，統一網址)
            → http://10.35.36.168:5062/    (lab-mac-mini，app 本體，launchd 常駐)
                → 讀 ~/Desktop/insight test log   (資料就在這台)

push deploy-to-itserver
   → lab-mac-mini 的 GitHub runner → checkout → rsync 到 serving 目錄 → uv sync → 重啟
```

---

## 前置需求（開始前先備齊）

- [ ] 公司內網 WiFi `L300301`（內網 10.x IP 才連得到）
- [ ] lab-mac-mini SSH：`ssh cpdx_sw@10.35.36.168`
- [ ] it-server SSH + sudo：`ssh jason@10.35.40.81`（Step 6 改 nginx 用）
- [ ] GitHub repo admin（加 Deploy key + 拿 runner token）
- [ ] lab-mac-mini 已裝 uv（`which uv`，沒有就 `curl -LsSf https://astral.sh/uv/install.sh | sh`）
- [ ] 工具依賴的資料已在 lab-mac-mini（atlas 為 `~/Desktop/insight test log`）
- [ ] 能 GUI 操作 lab-mac-mini（Step 3 設權限用）：`open vnc://10.35.36.168`

> 下面以 atlas 為例，換成你自己的 repo / port / 路徑即可。`<...>` 是要替換的地方。

---

## Step 1 — 設 deploy key（一定要最先做）

lab-mac-mini 的 git 有一條全域設定會把 GitHub 的 HTTPS 網址改寫成 SSH。不先處理，後面 clone 和 runner 都會用到沒權限的 key、拉不到 code（詳見文末註記 ②）。給這個 repo 一把專屬 deploy key：

```bash
# on lab-mac-mini
ssh-keygen -t ed25519 -f ~/.ssh/atlas_deploy_key -N '' -C 'atlas deploy key'
cat >> ~/.ssh/config <<'EOF'

Host github-atlas
    HostName github.com
    User git
    IdentityFile ~/.ssh/atlas_deploy_key
    IdentitiesOnly yes
EOF
# 只讓這個 repo 的網址走專屬 key（不動全域設定）
git config --global url.'git@github-atlas:fitfabsw/atlas-log-wizard'.insteadOf \
  'https://github.com/fitfabsw/atlas-log-wizard'
cat ~/.ssh/atlas_deploy_key.pub        # ← 複製這串
```

把 pub key 加到 GitHub：**repo → Settings → Deploy keys → Add**（read-only，不勾 write）。

驗證（要回 commit sha 才算通）：

```bash
git ls-remote https://github.com/fitfabsw/atlas-log-wizard.git deploy-to-itserver
```

## Step 2 — 拉 code 到 serving 目錄、建 venv

serving 目錄放 `~/` 第一層（避開 macOS 隱私限制，見註記 ①）。clone 到暫存再 rsync（不帶 `.git`，跟自動部署一致）：

```bash
# on lab-mac-mini
git clone -b deploy-to-itserver https://github.com/fitfabsw/atlas-log-wizard.git /tmp/atlas-src
rsync -a --exclude='.git' /tmp/atlas-src/ ~/atlas-log-wizard/ && rm -rf /tmp/atlas-src
cd ~/atlas-log-wizard
uv venv .venv && uv sync
cp config.example.json config.json     # 需要 AI 功能才填，見 Step 7；只看 log 可留空
```

## Step 3 — 開「完整取用磁碟權」（讓背景服務讀得到資料）

macOS 會擋背景服務讀 `~/Desktop`（見註記 ①）。用螢幕共享進去設定：

1. `open vnc://10.35.36.168`（cpdx_sw 帳密）
2. **系統設定 → 隱私權與安全性 → 完整取用磁碟權 → `+`**
3. `Cmd+Shift+G` 貼**真實 python 路徑**（用下面指令查，不是 `.venv` 那個 symlink）：
   ```bash
   readlink -f ~/atlas-log-wizard/.venv/bin/python
   ```
4. 把它打勾啟用

> 若你的資料本來就放在 `~/` 第一層（非 Desktop/Documents/Downloads），這步可跳過。

## Step 4 — launchd 常駐

把 repo 內附的 launchd 設定檔載入，讓 app 開機自啟、常駐：

```bash
# on lab-mac-mini
cp ~/atlas-log-wizard/deploy/com.cpdx.atlas.plist ~/Library/LaunchAgents/
launchctl load ~/Library/LaunchAgents/com.cpdx.atlas.plist
sleep 3; lsof -nP -iTCP:5062 -sTCP:LISTEN | grep -q LISTEN && echo '✓ :5062 起來了'
```

> 之後要手動重啟（改了 config 等）：`launchctl kickstart -k gui/$(id -u)/com.cpdx.atlas`

## Step 5 — 裝 GitHub runner（push 自動部署）

```bash
# on lab-mac-mini
mkdir -p ~/actions-runner-atlas && cd ~/actions-runner-atlas
curl -sL -o r.tgz https://github.com/actions/runner/releases/download/v2.335.1/actions-runner-osx-arm64-2.335.1.tar.gz
tar xzf r.tgz && rm r.tgz
# token：repo → Settings → Actions → Runners → New self-hosted runner（macOS/Arm64）
#        ⚠️ token 短效（約 1 小時），拿到就立刻用
./config.sh --url https://github.com/fitfabsw/atlas-log-wizard \
  --token <RUNNER_TOKEN> --labels lab-mini --name lab-mac-mini-atlas \
  --work _work --unattended --replace
./svc.sh install && ./svc.sh start
```

驗證：push 一個 commit 到 `deploy-to-itserver`，到 repo 的 **Actions** 分頁看是否綠勾、`~/atlas-log-wizard` 是否更新。

## Step 6 — it-server 反代（對外統一網址）

把 repo 內附的 nginx 設定推上 it-server 套用：

```bash
scp ~/atlas-log-wizard/deploy/atlas.conf jason@10.35.40.81:/tmp/
ssh -t jason@10.35.40.81 'sudo cp /tmp/atlas.conf /etc/nginx/app-locations/ && sudo nginx -t && sudo systemctl reload nginx && curl -s -o /dev/null -w "atlas → %{http_code}\n" http://127.0.0.1/atlas/'
```

看到 `200` 就成了 —— 對外 `http://10.35.40.81/atlas/` 可用。

## Step 7（選用）— AI 功能的 LLM 設定

要用 AI 分析才需要。編輯 `~/atlas-log-wizard/config.json` 填入 LLM 服務（atlas 接 lab-mac-mini 本機的 AnythingLLM :8888），填好後重啟服務（Step 4 的 kickstart）。

---

## 完成後的日常

**改 code → push → 自動上線。** 就這樣。

只有改部署設定（launchd plist / nginx conf）時，才需要重做 Step 4 / Step 6 的套用指令。

> 完整且持續更新的指令 SOP 在 repo 內：[`deploy/README.md`](https://github.com/fitfabsw/atlas-log-wizard/blob/deploy-to-itserver/deploy/README.md)。

---

## 附錄：踩雷註記

照上面做通常就沒事，但這幾個雷踩到會很難 debug，列在這供查：

**① macOS TCC —— 背景服務讀不到 ~/Desktop**
TCC 保護 `~/Desktop`/`~/Documents`/`~/Downloads`。你手動跑讀得到（繼承 GUI 同意），但 **launchd 背景服務讀不到，而且是直接 hang 住不報錯**。解法：開「完整取用磁碟權」給**真實 python**（Step 3），或把資料搬到 `~/` 第一層。授權真實路徑（非 `.venv` symlink）才不會被 `uv sync` 重建 venv 弄失效。

**② runner checkout 拉到空 repo（最隱蔽）**
lab-mac-mini 全域 git 有 `url.ssh://git@github.com/.insteadOf https://github.com/`，把 `actions/checkout` 的 HTTPS+token 網址改寫成 SSH → 用到沒權限的預設 key → checkout 只 `git init` 空 repo、**job 卻顯示 success、部署到舊 code**。解法就是 Step 1 的專屬 deploy key + 精確 insteadOf。**每個在 lab-mac-mini 裝 runner 的 repo 都要做。**

**③ 瀏覽器封鎖某些 port**
5060/5061（SIP）、6000、6666 等被瀏覽器內建黑名單擋（`ERR_UNSAFE_PORT`）；macOS 5000 被 AirPlay 佔。挑 dev port 要避開，atlas 用 5062。

**④ Flask debug 不能上 prod**
`debug=True` 會外露 Werkzeug debugger（可遠端執行程式碼）。改成 `--debug` 旗標、預設關。

**⑤ 不要直接跑 checkout 目錄**
`actions/checkout` 預設 `git clean` 會砍掉 `config.json`/`.venv`；且邊部署邊跑會打架。所以要 rsync 到獨立 serving 目錄（`--exclude` 保住 config/venv），讓 launchd 跑那個穩定目錄。

---

## 何時該用這套

| | it-server 直接部署（預設）| lab-mac-mini + 反代（本文）|
|---|---|---|
| 適用 | app 沒綁特定機器 | **app 依賴某台機器的本機資料/硬體** |
| 複雜度 | 低 | 高（TCC、deploy key、launchd…）|
| 可靠度 | 伺服器，穩 | 共用 Mac，可能睡眠/被人動 |

新工具預設 it-server；只有被資料/硬體綁死在某台機器時才用這套。別把關鍵服務長期壓在共用 Mac 上。

---

*有部署疑問，或想把自己的工具接上自動部署，歡迎找我討論。*
