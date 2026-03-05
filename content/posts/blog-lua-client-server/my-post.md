---
title: "Simple client server using Lua"
date: 2026-03-05
author: Simon Chen
description: "簡單的Lua client server"
tags: ["lua", "TCP", "code"]
math: true
ShowToc: true
TocOpen: true
---

本文示範簡單的Client Server code for Lua。

<!--more-->

## Client

{{< highlight lua >}}
local socket = require("socket")

-- Connect to localhost on port 12345
local client = socket.tcp()
client:connect("127.0.0.1", 12345)

-- Send data
client:send("Hello Ethernet!\n")

-- Receive response
local line, err = client:receive()
if not err then
    print("Received: " .. line)
end

client:close()
{{< /highlight >}}

## Server

{{< highlight lua >}}
local socket = require("socket")

-- Bind to all interfaces on port 12345
local server = assert(socket.bind("*", 12345))
local ip, port = server:getsockname()
print("Waiting for connections on " .. ip .. ":" .. port)

-- Accept a client connection
local client = server:accept()
client:send("Welcome to Lua Server\n")

local line = client:receive()
print("Client said: " .. line)

client:close()
server:close()
{{< /highlight >}}
