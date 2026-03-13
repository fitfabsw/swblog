---
title: "Hyperion vs Station.rush vs loadExample"
date: 2026-03-13
author: Simon Chen
description: "比較Hyperion.app和Station.rush和loadExample"
tags: ["Atlas1", "Hyperion", "rush"]
math: true
ShowToc: true
TocOpen: true

cover:
  image: "h.png"           # 圖片路徑（本地或 URL）
  alt: "Hyperion"              # 圖片替代文字
  caption: "Atlas1 Hyperion"      # 圖片標題
  responsiveImages: false  # 保持原始比例
---

## Hyperion vs Station.rush vs loadExample
Station.rush 不是單純「被 Hyperion.app 直接執行」；
而是由 Atlas core 搭配 Rush runtime 載入與執行，Hyperion.app 則是啟動／連接這個 station 的應用程式介面。

我拆成兩層講：

### 1. 誰真的在跑 Station.rush？

在範例 Station.rush 註解裡，明確寫到：

- app_setup(app)
當 app 第一次連到 atlas core，且 station 尚未啟動前被呼叫

- station_setup()
當 core 第一次啟動 station 時被呼叫

- unit_setup(unit)
當 core 收到新 unit 時被呼叫

所以從文件語意來看，真正觸發這些 callback 的是 Atlas core。
而 Rush 本身也有獨立 runtime / CLI，可直接跑 .rush：

- /AppleInternal/Library/Frameworks/Rush.framework/bin/rush

- rushc

- rushl

- rushr

這代表 Rush 是執行語言/runtime，本身不依賴 Hyperion 才能存在。

### 2. 那 Hyperion.app 的角色是什麼？

在 iOS tutorial 裡，文件流程是：

I. 先執行 ./loadExample

II. 再 open /AppleInternal/Applications/Hyperion.app

III. 插入 unit 後，Hyperion 會啟動 unit sequencer 並開始測試

另外文件還寫：

Hyperion functions much like an Atlas plugin...

也就是說，Hyperion 比較像 Atlas 的前端 app / UI 容器，會和 Atlas core 互動。
你在 Station.rush 裡設定的 app.settings（例如 "station-name"、"start-button"、"station-topology"）也是給這類 app 使用的。

所以你的問題，最接近的答案是
Station.rush 是由哪個程式呼叫執行？

底層是 Atlas core + Rush runtime 在執行。

是 Hyperion.app 呼叫嗎？

在一般 Atlas 範例操作流程中，通常是你打開 Hyperion.app，由 Hyperion 去連到 Atlas core，然後 core 再載入並執行 Station.rush。

所以：

- 若問「使用者是開哪個程式來跑 station？」
常見答案是：Hyperion.app

- 若問「真正執行 Station.rush 的執行引擎是誰？」
比較正確是：Atlas core / Rush runtime

- 若問「是不是只有 Hyperion 才能跑？」
不一定。文件語意顯示，只要有 app 連到 atlas core，都可能觸發這套流程；Hyperion 只是官方範例裡最常見的那個 app。

你可以把它想成：

Hyperion.app = 前台/UI　<br>
Atlas core = station 生命周期管理者 <br>
Rush runtime = 實際執行 Station.rush 的語言引擎 <br>
