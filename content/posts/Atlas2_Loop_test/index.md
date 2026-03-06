---
title: "Atlas2 Loop test"
date: 2026-03-05T10:35:31+08:00
author: "Alan Hung"
description: "Atlas2 loop的兩種實現方法"         # 搜尋引擎與社群分享用的描述
tags: ["Atlas2"]     # 標籤
categories: ["教學"]           # 分類
series: ["Atlas2 指南"]          # 系列文章
summary: "Atlas2 loop的兩種實現方法"            # 文章列表中顯示的摘要
draft: False                    # 草稿，設為 true 則不會在正式建置中顯示

# 目錄
ShowToc: true                  # 顯示目錄
TocOpen: true                  # 預設展開目錄

# 封面圖片
cover:
  image: "img/brain_loading.jpeg" # 圖片路徑（本地或 URL）
  alt: "Atlas2"              # 圖片替代文字
  caption: "OK"  

---

## 1. 簡介

在 Atlas2中，要讓測項重複執行（迴圈），最直接的方法就是修改 **`Loop`** 欄位。

要讓所有測項執行 1000 輪，你有兩種方法：

### 1：**直接在 CSV 中設定 Loop 值**
- 在main.csv 的 `Loop` 欄位中填入 `1000`
- 這樣每個測項就會執行 `1000` 次
<div style="display: flex; justify-content: center;">
  <figure style="margin:0; justify-content: center;">
    <img src="./img/1.png" alt="圖一" style="max-width:80%; height:auto; display:block; justify-content: center;">
  </figure>
</div>

- 但此時是測項1執行1000次後才接續執行測項2

- 全部執行要在所有測項加的 `Loop` 欄位中填入 `1000`

這可能不符合我們想要的

### 2：**使用外層迴圈控制**
- 在 Lua 的序列控制或狀態機中，用迴圈包裝整個測試序列
- 這樣可以更靈活地控制迴圈邏輯
- 其中：
1. **`loopAgain()` 函數** - 控制整個測試序列是否重複
2. **`loops_per_detection` 變數** - 在 `userPluginModule` 中定義（Station/Plugins.lua）
3. **`unitDetection()` 函數** - 設定每個單位檢測週期應該執行多少輪（Matchbox/group.lua）


在 `Plugins.lua` 中設定 `loops_per_detection = 1000`。

<div style="display: flex; justify-content: center;">
  <figure style="margin:0; justify-content: center;">
    <img src="./img/2.png" alt="圖二" style="max-width:80%; height:auto; display:block; justify-content: center;">
  </figure>
</div>

- 這個設定會讓測試系統每次檢測到一個單位後，自動執行 Init → Main → Teardown 的完整測試序列 1000 次
- 系統會透過 `loopAgain()` 函數來控制是否進行下一輪，當 `loopsLeft` 大於 0 時繼續循環

實現做 1000 次的功能（待驗證）
<div style="display: flex;">
  <figure style="margin:0; justify-content: center;">
    <img src="./img/mur貓.gif" alt="開搖" style="max-width:80%; height:auto; display:block; justify-content: center;">
  </figure>
</div>

---


