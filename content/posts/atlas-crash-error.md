---
title: "Atlas2 閃退問題：如何找到 Crash Report"
date: 2026-04-23T13:51:37+08:00
author: "Ted Yen"
description: "Atlas2 UI 發生閃退時，macOS 會自動產生診斷報告，本文說明如何找到這份檔案並提供給開發團隊分析。"
draft: false
tags: ["Atlas2"]
categories: ["教學"]
series: ["Atlas2 指南"]
summary: "Atlas2 閃退後 macOS 會自動產生 Crash Report，本文說明如何在 DiagnosticReports 資料夾中找到它。"
ShowToc: true
TocOpen: true
---

## 前言

當 Atlas2 發生非預期閃退時，程式來不及完成正常的關閉流程，因此 **不會產生 `device.log`**。這種情況下，唯一能取得診斷資訊的方式，是透過 macOS 在閃退當下自動記錄的診斷報告（Crash Report）。

這份報告包含閃退當下的呼叫堆疊（call stack）、記憶體狀態等資訊，是開發團隊分析問題根源的重要依據。

---

## 閃退報告的位置

macOS 將所有應用程式的閃退報告統一存放在以下路徑：

```
~/Library/Logs/DiagnosticReports
```

其中 `~` 代表你的使用者家目錄，完整路徑為：

```
/Users/<你的使用者名稱>/Library/Logs/DiagnosticReports
```

---

## 如何開啟這個資料夾

### 方法一：Terminal 指令（最快）

開啟 Terminal，貼上以下指令，按 Enter：

```bash
open ~/Library/Logs/DiagnosticReports
```

Finder 會自動開啟該資料夾。

### 方法二：Finder 前往

1. 開啟 **Finder**
2. 從選單列點選 **「前往」→「前往檔案夾…」**（或按 `⇧⌘G`）
3. 輸入路徑後按 Enter：
   ```
   ~/Library/Logs/DiagnosticReports
   ```

---

## 找到 Atlas2 的閃退報告

進入資料夾後，找檔名開頭為 `Atlas2` 的 `.ips` 或 `.crash` 檔案，依時間排序後取最新一筆即為本次閃退的報告。

```
Atlas2_2026-04-23-143012_MacBook-Pro.ips
```

---

## 回報給開發團隊

請將該檔案附上提供給開發團隊，並補充以下資訊，有助於加快問題定位：

- 閃退發生的時間
- 閃退前正在操作的功能或步驟
- 是否為特定操作必定重現，或隨機發生
