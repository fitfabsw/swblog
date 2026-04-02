---
title: "No unsafe page on Chrome"
date: 2026-03-05
author: Simon Chen
description: "用Chrome連https沒任何反應"
tags: ["Chrome", "Curl", "卡巴斯基"]
math: true
ShowToc: true
TocOpen: true

cover:
  image: "c.png"           # 圖片路徑（本地或 URL）
  alt: "chrome"              # 圖片替代文字
  caption: "curl error"      # 圖片標題
  responsiveImages: false  # 保持原始比例
---

用Chrome, 連接https://<內網IP>時，
不會出現unsafe page,
而是直接完全沒反應，
後來發現，是卡巴斯基軟体擋掉。

<!--more-->

## 用Curl釐清
fit0721@FIT0721deMacBook-Pro ~ % curl -vk --http1.1 https://10.35.40.81/swblog/ * Trying 10.35.40.81:443... * Connected to 10.35.40.81 (10.35.40.81) port 443 * ALPN: curl offers http/1.1 * (304) (OUT), TLS handshake, Client hello (1): * Recv failure: Operation canceled * LibreSSL/3.3.6: error:02FFF059:system library:func(4095):Operation canceled * Closing connection curl: (35) Recv failure: Operation canceled fit0721@FIT0721deMacBook-Pro ~ % curl -vk --http2 https://10.35.40.81/swblog/ * Trying 10.35.40.81:443... * Connected to 10.35.40.81 (10.35.40.81) port 443 * ALPN: curl offers h2,http/1.1 * (304) (OUT), TLS handshake, Client hello (1): * Recv failure: Operation canceled * LibreSSL/3.3.6: error:02FFF059:system library:func(4095):Operation canceled * Closing connection * Recv failure: Operation canceled curl: (35) Recv failure: Operation canceled

## 可能解法

如果出現以上Curl的失敗，
會造成Chrome用https連網頁時，
連unsafe page的頁面都不出現，
而沒有任何反應。
此時，有可能是軟体擋住，例如：卡巴斯基之類的軟体。
嘗試關掉卡巴斯基之類的軟体，
再用Chrome去連https之類的網頁，
應該就可以看到unsafe的頁面了。
