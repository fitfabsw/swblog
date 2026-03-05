---
title: "Atlas 2 量產測試中的 AI 導入藍圖：不改流程的側車式升級"
date: 2026-03-05

author: "Kenny"            
description: "文章摘要"         # 搜尋引擎與社群分享用的描述
tags: ["AI","Atlas2","量產測試","代工廠","製造數位化","RAG","PHM","電腦視覺","資料治理"]
categories: ["測試"]
series: ["Hugo 指南"]          # 系列文章
aliases: ["/old-url"]          # URL 別名，用於舊文章重導向
summary: "在不改動 Atlas 2 與 Apple 客戶規範的前提下，透過側車服務與資料治理，實現測試開發效率、日誌除錯、良率/節拍、設備健康與電腦視覺的可量化提升。"
draft: false                    

# 目錄
ShowToc: true                  # 顯示目錄
TocOpen: true                  # 預設展開目錄

cover:
  image: "ai.jpg"           # 圖片路徑（本地或 URL）
  alt: "ai"              # 圖片替代文字
  caption: "圖片說明"          # 圖片標題
  responsiveImages: false # 保持原始比例



# 顯示控制
ShowBreadCrumbs: true          # 顯示麵包屑導覽
hideMeta: false                 # 隱藏文章元資料（日期、作者等）
searchHidden: false             # 從搜尋中隱藏

# 數學公式
math: false 


---


### Atlas 2 量產測試中的 AI 導入藍圖：不改流程的側車式升級

> 關鍵結論
> 不把 AI 塞進 Atlas 2；讓 Atlas 2 持續擔任測試編排主控，AI 以「旁路側車」輔助層帶來效率與洞察。
> 先做「資料結構化與治理」→ 再做「AI 決策輔助」→ 最後才少量「閉迴路自動化（保留人審）」。
> 全程內網推論與資料脫敏，確保不影響客戶合規與必測流程。

## 為什麼現在導入 AI（而不改動 Atlas 2）

- 量測與日誌資料已經存在，只是分散且未結構化；AI 的價值在於「更快定位根因與給出可操作建議」。
- 測試腳本開發重複性高、受制於人力經驗；AI 可加速產生骨架與參數檢查，降低低階錯誤。
- 良率/節拍與設備健康可藉由歷史資料趨勢與異常樣態，做提早預警與優化，且不需碰觸必測邏輯。

## 可落地的 AI 場景（圍繞 Atlas 2 的輔助層）

1. 測試開發 Copilot

  - 從需求單自動生成 Atlas 2 測試樣板與檔案結構。
  - Timeout/上下限/通訊設定的靜態檢查與修正建議。
  - 讀儀器手冊/既有驅動，給常用指令與封裝雛形。

2. 日誌與除錯

  - 即時異常偵測與根因摘要（針對串口回應、量測時序、Atlas 2 log）。
  - FAIL 記錄聚類成 N 類典型問題，鏈接 KEDB（已知錯誤庫）。
  - 中英雙語摘要，支援跨團隊協作。

3. 良率與節拍

  - 測項排序與門檻優化的候選清單（人審後採用）。
  - 預測重測/返修率，提早調度工單與備件。
  - 線平衡/排程建議，降低等待與瓶頸。

4. 設備健康（PHM）

  - 以通訊錯誤率、回應延遲、校準飄移做維護預測。
  - 站點 MTBF/MTTR 與飄移趨勢面板。

5. 電腦視覺（外觀/銘板/OCR）

  - 輕量模型在工控機本地推論；提供熱點圖輔助人員複核。
  - 雷雕/標籤文本校驗與定位。

6. 知識與自動化

  - RAG 知識庫：SOP、BOM、手冊、Atlas 2 範例可檢索。
  - RPA：與 MES/倉儲/標籤軟體做自動化互動。


# 與既有流程的銜接（側車整合，不動 Atlas 2）

- 事件匯流：Atlas 2 測試結束後輸出 JSON/CSV/SQLite 到本機資料夾或經由 TCP/REST；側車服務收集、清洗、入庫。

- 最小資料模式（建議欄位）
  <mark>station_id, fixture_id, product, sku, unit_sn, test_step, meas_name, value, lo, hi, pass, start_ts, end_ts, fw_version, sw_build, operator_id, error_code, raw_log_ptr</mark>

  - 推論位置：內網/on‑prem 小模型（LLM + 輕量 CV），資料預設脫敏。
  - 回饋方式：面板/訊息告警/每日收斂報告/自動開單（保留審核流程）。

 :robot::robot::robot::robot::robot::robot:


{{< figure src="fit.png" alt="範例圖片" caption="這是圖片說明" >}}
