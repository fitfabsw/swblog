---
title: "Toni_note"
date: 2026-03-05
author: "Toni Lin"            # 作者，也可用陣列：["Kevin Lee", ""]
description: "AI mission"      # 搜尋引擎與社群分享用的描述
tags: ["hugo", "markdown"]    # 標籤
categories: ["教學"]           # 分類
series: ["Hugo 指南"]          # 系列文章
aliases: ["/old-url"]         # URL 別名，用於舊文章重導向
summary: "AI"                 # 文章列表中顯示的摘要
draft: true    

---
以下為AI可應用於測試工站的研發想定,後續可再增加

1.AI幫助專業感測器測站開發-
利用AI  工具(Copilot)抓出過往log file ,制定新的通訊協定與專業計算值
技術繞道校正值計算,因爲客人在原先atlas1 overlay’s library 無法移植到atlas2

2.縮短測試站test time & retest rate-
利用AI 修改overlay 內CSV & Lua ,擬合新的test item(optimization),
 
—>可用外部網路,VPN至內網遠端操作與修改條件
—>Run OVL on Intel Mac ,not J174



3.自動化工站監測-標準化設備製造及監控部件,可知其使用次數及狀態
抓出 AI工具整合成新的APP,與OVL並存執行,下位機有兩個USB ports,其一與OVL通訊,                                                             其一監控儀器部件狀態(Weak-AI),先有資料才能進行更難的AI處理


MCU 1 to USB_01  ,connect to Mac USB1 ,Atlas2 Overlay
MCU 2 to USB_02 ,connect to Mac USB2 ,Monitor Overlay 
   Predict the fail issue ,execute System reset or inform 