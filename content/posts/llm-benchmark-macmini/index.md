---
title: "MacMini 開源大語言模型效能評估"
date: 2026-03-05
author: Peter
description: "評估開源 LLM 在 MacMini 及邊緣裝置上的推論效能與可行性"
tags: ["llm", "benchmark", "edge-ai"]
draft: true
ShowToc: true
TocOpen: true
---

評估開源 LLM 在 MacMini 及邊緣裝置上的執行效能與可行性，作為後續應用的技術參考依據。

<!--more-->

## 階段一：環境建置

- 在 MacMini 上安裝推論框架（如 Ollama、LM Studio、llama.cpp、vLLM）
- 確認硬體規格並記錄（晶片型號、記憶體、儲存空間）

## 階段二：模型測試

選定數個主流開源模型，測試不同參數量版本：

_Examples_

**重點測試模型：Qwen 3.5**（最新推出，優先完整測試各參數版本）

| 模型            | 參數版本（建議）         | 優先度 |
| --------------- | ------------------------ | ------ |
| **Qwen 3.5**    | **全系列參數版本**       | **高** |
| Llama 4         | Scout 17B / Maverick 17B | 中     |
| Gemma 3         | 4B / 12B / 27B           | 中     |
| Phi 4           | Mini 3.8B / 14B          | 中     |
| Mistral Small 3 | 24B                      | 低     |

（可依實際記憶體限制調整）

## 階段三：Benchmark 評估

針對每個模型記錄以下指標：

- **推論速度：** tokens/sec（首 token 延遲 + 持續生成速度）
- **記憶體用量：** 峰值 RAM / VRAM 占用
- **量化比較：** FP16 vs Q8 vs Q4 對速度與品質的影響
- **回應品質：** 針對中英文問答、程式碼生成、摘要等任務做簡單評分

## 階段四：跨裝置比較（選做）

- 在樹莓派或其他邊緣裝置上部署小型模型（≤ 3B）
- 比較與 MacMini 的效能差異
- 評估邊緣裝置部署的可行性與限制

## 產出

1. 一份評估報告，包含各模型 × 各裝置的 benchmark 數據表
2. 結論與建議：哪些模型 / 參數量適合在哪些裝置上跑、適合什麼應用場景
