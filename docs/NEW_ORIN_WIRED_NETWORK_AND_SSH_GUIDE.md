# 新 Armstrong Orin 有线网络、SSH 密钥与多网口基础指南

最后核对日期：2026-08-21

本文面向第一次接触该系统的同事，说明如何让 Windows 笔记本通过网线连接新机器人 Orin，如何配置 SSH 私钥，以及 Orin、机械臂控制器、双 4090 主机、交换机、多网口、路由和 ROS 2 Domain ID 之间的关系。

本文只讨论网络和远程登录。执行本文中的检查命令不会给机器人上电、使能或发送运动命令。

> 安全原则：网络连通不等于允许机器人运动。SSH 登录、Ping 成功、ROS 2 节点可见都不会替代实体急停、限位和软件运动门。

## 1. 当前地址与角色

| 设备 | 当前地址/名称 | 账号或作用 | 状态 |
| --- | --- | --- | --- |
| Windows 笔记本有线口 | `192.168.2.130/24` | ZLink2 读取、UDP 发送、SSH 启停 | 已确认 |
| Windows Wi-Fi | 地址由办公网络分配，例如 `192.168.0.105` | 日常上网 | 地址会变化 |
| 新机器人 Orin | `192.168.2.170` | Ubuntu 22.04、ROS 2 Humble、相机、机器人控制 | 已确认 |
| 新 Orin 主机名 | `nvidia-desktop` | SSH 登录后用于核对身份 | 已确认 |
| 新 Orin SSH 用户 | `nvidia` | SSH 账号 | 已确认 |
| Armstrong 控制器 | `192.168.2.226` | JAKA/Armstrong SDK 的 TCP/IP 控制目标 | 已确认当前项目使用此地址 |
| 双 4090 推理主机 | `192.168.2.110` | 只做模型推理，Orin 负责最终控制 | 已确认；用户为 `walle` |
| 新 Orin 推理侧候选地址 | `192.168.2.171/32` | 曾用于新 Orin 到 `.110` 的隔离链路 | 历史配置，重启后必须复核 |
| 旧 Ubuntu 主机 | `192.168.2.116` / Wi-Fi `192.168.0.36` | 旧 Jazzy 链路 | 不属于新 Orin 登录地址 |

不要把密码、私钥内容或服务器凭据提交到 GitHub。本仓库只记录地址、用户名、密钥文件名和配置方法。

## 2. 数据到底怎样流动

当前主从遥操作链路：

```text
ZLink2 主臂
    │ USB / COM10
    ▼
Windows 192.168.2.130
    ├── SSH/TCP 22 ───────────────► Orin 192.168.2.170
    └── 主臂 UDP/5005 ────────────► Orin 192.168.2.170
                                          │
                                          ├── 本机 ROS 2：桥接、相机、夹爪
                                          └── JAKA SDK/TCP ─► 控制器 192.168.2.226
```

LingBot/远端 4090 推理链路：

```text
相机 + 机器人观测
        │
        ▼
新 Orin（采集、ROS 2、安全门、控制）
        │ 有线模型请求/动作结果
        ▼
双 4090 主机 192.168.2.110（只推理）
        │
        └────────────► 结果返回 Orin ─► Orin 再控制 192.168.2.226
```

4090 主机不应直接替代 Orin 控制机械臂。这样即使推理服务异常，最终的限位、急停状态和执行接口仍留在机器人侧。

## 3. 网线、SSH、IP 和 ROS 2 不是同一件事

### 3.1 网线是什么

网线和交换机提供物理及二层连接。链路灯亮、Windows 显示 `1.0 Gbps`，只证明网卡协商成功，不证明 IP、路由或 SSH 已配置正确。

普通 Cat5e/Cat6 网线即可支持 1 Gbps。现代网卡通常支持自动翻转，不需要专门的交叉线。

### 3.2 IP 地址是什么

IP 地址用于在网络中找到一台主机。例如 `192.168.2.170/24` 中：

- `192.168.2.170` 是主机地址；
- `/24` 等价于掩码 `255.255.255.0`；
- `192.168.2.1` 到 `192.168.2.254` 通常被看作同一个 IPv4 子网；
- 同一二层网络内不能有两台同时工作的设备使用同一个 IP。

### 3.3 SSH 是什么

SSH 是运行在 IP 网络上的加密远程终端协议，默认 TCP 端口为 `22`。必须先有网线/Wi-Fi和正确 IP 路由，SSH 才可能连接。

- `ping` 通：只证明 ICMP 基本可达；
- TCP 22 通：证明 SSH 服务端口可达；
- SSH 密钥验证成功：才能真正登录；
- SSH 登录成功：仍不表示机器人已经上电或使能。

PowerShell 和 MobaXterm 都只是 SSH 客户端。PowerShell 自带的 `ssh.exe` 已足够使用；MobaXterm 额外提供会话管理、SFTP 文件树和 X11 转发。

### 3.4 ROS_DOMAIN_ID 是什么

`ROS_DOMAIN_ID` 只隔离 ROS 2 DDS 节点发现和 ROS 2 Topic/Service。它不能隔离：

- SSH；
- UDP/5005；
- 机械臂 SDK 到 `192.168.2.226` 的 TCP 连接；
- 两台相同 IP 设备造成的 ARP 冲突。

因此，两台机器人的控制器都为 `192.168.2.226` 时，只设置不同 `ROS_DOMAIN_ID` 不足以保证安全隔离。

临时设置方法：

```powershell
# Windows：只对当前 PowerShell 窗口生效
$env:ROS_DOMAIN_ID = "31"
```

```bash
# Ubuntu：只对当前 Bash 生效
export ROS_DOMAIN_ID=31
```

所有需要互相发现的 ROS 2 进程必须使用同一个 Domain ID。

## 4. Windows 第一次有线连接新 Orin

### 4.1 接线

1. 机器人 Orin 开机。
2. Windows 网线插入机器人所在交换机或指定的 Orin 接入口。
3. 查看交换机端口灯和 Windows 以太网状态。
4. 不要随意把原本物理隔离的两套机器人交换机互连。

### 4.2 设置 Windows 静态 IPv4

推荐在图形界面设置：

`设置 → 网络和 Internet → 高级网络设置 → 更多网络适配器选项 → 以太网 → 属性 → Internet 协议版本 4 (TCP/IPv4)`

填写：

```text
IP 地址：192.168.2.130
子网掩码：255.255.255.0
默认网关：留空（机器人隔离网推荐）
DNS：留空（机器人隔离网推荐）
```

为什么推荐网关和 DNS 留空：

- 机器人网只负责访问 `192.168.2.x`；
- Wi-Fi 继续负责互联网；
- 避免 Windows 错把互联网流量发给机器人交换机；
- 避免 Ethernet 和 Wi-Fi 同时提供默认路由导致偶发断连。

如果现场 `192.168.2.1` 确实是经过管理员配置的路由器，并且明确要求通过它联网，才填写网关。当前这台笔记本曾配置过 `192.168.2.1` 和 `8.8.8.8`，但它们不是访问 Orin 所必需的。

### 4.3 查看 Windows 网卡

```powershell
Get-NetAdapter |
  Format-Table Name, Status, LinkSpeed, MacAddress, ifIndex -AutoSize

Get-NetIPAddress -AddressFamily IPv4 |
  Format-Table InterfaceAlias, IPAddress, PrefixLength, AddressState -AutoSize

Get-NetIPConfiguration
```

期望看到：

- 以太网为 `Up`；
- 以太网地址为 `192.168.2.130/24`；
- 协商速率通常为 `1 Gbps`；
- Wi-Fi 仍可保持连接。

`Deprecated` 常出现在网线拔掉后保留静态地址的情形。重新接线并等待网卡变为 `Up` 后再检查。

### 4.4 强制验证走有线

```powershell
ping -S 192.168.2.130 192.168.2.170
Test-NetConnection 192.168.2.170 -Port 22
route print -4
```

普通 `ping 192.168.2.170` 可能由 Windows 自行选源地址。使用 `-S 192.168.2.130` 可以明确验证有线路径。

对当前遥操作脚本而言，Windows 必须到达 Orin `.170`。Windows 不一定必须直接到达控制器 `.226`，但 Orin 必须能够到达 `.226`。

## 5. 第一次 SSH 密码登录

Windows 10/11 通常自带 OpenSSH Client：

```powershell
ssh -V
```

首次连接：

```powershell
ssh nvidia@192.168.2.170
```

第一次会显示主机 ED25519 指纹并询问：

```text
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

先与设备负责人核对指纹或至少确认当前网线只连接目标机器人，然后输入 `yes`。随后输入现场保存的账号密码。密码输入时终端不会显示星号，这是正常行为。

登录后立即核对身份：

```bash
hostname
hostname -I
cat /etc/os-release | head
```

当前目标应显示主机名 `nvidia-desktop`、Ubuntu 22.04，并包含 `192.168.2.170`。

退出：

```bash
exit
```

## 6. 配置 SSH 私钥

### 6.1 密钥的组成

```text
one_arm_teleop_ed25519       私钥：只能保存在受信任电脑上，绝不能发给别人或提交 Git
one_arm_teleop_ed25519.pub   公钥：可以安装到 Orin 的 authorized_keys
```

当前项目约定的 Windows 私钥路径：

```text
C:\Users\<Windows用户名>\.ssh\one_arm_teleop_ed25519
```

### 6.2 生成新密钥

如果电脑还没有该文件：

```powershell
New-Item -ItemType Directory -Force "$env:USERPROFILE\.ssh" | Out-Null
ssh-keygen -t ed25519 `
  -f "$env:USERPROFILE\.ssh\one_arm_teleop_ed25519" `
  -C "one-arm-teleop"
```

建议设置密钥口令；需要一键脚本无人值守时，可结合 Windows `ssh-agent` 管理口令，而不是把密码写进脚本。

### 6.3 安装公钥到 Orin

Windows PowerShell 没有 `ssh-copy-id` 也可以安装：

```powershell
Get-Content "$env:USERPROFILE\.ssh\one_arm_teleop_ed25519.pub" |
  ssh nvidia@192.168.2.170 `
  "umask 077; mkdir -p ~/.ssh; cat >> ~/.ssh/authorized_keys; chmod 600 ~/.ssh/authorized_keys"
```

此步骤会要求输入一次 Orin 密码。它只上传 `.pub` 公钥，绝对不要把不带 `.pub` 的私钥复制到 Orin。

### 6.4 限制 Windows 私钥权限

```powershell
$key = "$env:USERPROFILE\.ssh\one_arm_teleop_ed25519"
icacls $key /inheritance:r
icacls $key /grant:r "$($env:USERNAME):(R)" "SYSTEM:(F)" "Administrators:(F)"
```

不同语言版本 Windows 的管理员组名称可能不同。如果 `Administrators` 无法识别，可只保留当前用户与 `SYSTEM`，或在文件属性的“安全”页面中移除无关用户。

### 6.5 配置 SSH 别名

编辑：

```text
C:\Users\<Windows用户名>\.ssh\config
```

加入：

```sshconfig
Host armstrong-orin
    HostName 192.168.2.170
    User nvidia
    Port 22
    IdentityFile C:/Users/<Windows用户名>/.ssh/one_arm_teleop_ed25519
    IdentitiesOnly yes
    ServerAliveInterval 30
    ServerAliveCountMax 3
```

注意 SSH config 中推荐使用 `/`。不要把 `<Windows用户名>` 原样保留。

验证无密码/无交互登录：

```powershell
ssh -o BatchMode=yes armstrong-orin "hostname; hostname -I"
```

成功时应输出 `nvidia-desktop`。`BatchMode=yes` 失败时不会退回密码提示，适合检查一键脚本需要的密钥是否正常。

当前这台开发电脑已存在 `armstrong-orin` 别名和对应私钥；新电脑仍需按本节重新配置。

### 6.6 MobaXterm 配置

新建 SSH Session：

```text
Remote host: 192.168.2.170
Specify username: nvidia
Port: 22
```

进入 `Advanced SSH settings`，勾选 `Use private key`，选择：

```text
C:\Users\<Windows用户名>\.ssh\one_arm_teleop_ed25519
```

MobaXterm 可以直接使用 OpenSSH ED25519 私钥，通常不需要转换为 `.ppk`。普通终端操作不需要 X11 forwarding；只有运行远端 GUI 并把窗口显示到 Windows 时才需要它。

## 7. Orin 多网口怎样判断

不要凭记忆把接口写死为 `eth0`、`eth4`、`eth6` 或 `eth7`。接口名可能因内核、扩展网卡和插槽变化而改变。

在 Orin 上只读检查：

```bash
hostname
ip -br link
ip -br addr
ip route
nmcli device status
nmcli -t -f NAME,DEVICE,TYPE connection show --active
```

查某个目标实际走哪个接口：

```bash
ip route get 192.168.2.226
ip route get 192.168.2.110
ip route get 192.168.2.130
```

输出示例：

```text
192.168.2.110 dev eth0 src 192.168.2.171
```

含义是：到 `.110` 的包从 `eth0` 发出，并使用 `.171` 作为源地址。

### 7.1 物理确认接口的方法

1. 记录 `ip -br link`。
2. 一次只拔一根非关键网线。
3. 再运行 `ip -br link`，观察哪个接口从 `LOWER_UP` 变为 `NO-CARRIER`。
4. 立即插回，并在标签纸上记录“接口名—网线—对端设备”。

不要在机器人运动时拔网线做确认。

也可以使用：

```bash
sudo ethtool eth0 | grep -E 'Speed|Duplex|Link detected'
cat /sys/class/net/eth0/address
```

MAC 地址属于网卡接口，IP 地址属于软件配置。交换机主要根据 MAC 地址转发二层帧。

## 8. 交换机、路由与 `/32` 主机路由

### 8.1 普通交换机做什么

普通以太网交换机工作在二层：

- 学习各端口后面的 MAC 地址；
- 将单播帧转发到相应端口；
- 将 ARP、广播等发到同一广播域的其它端口；
- 通常不会自动修改 IP，也不会替两台同 IP 设备做隔离。

如果两个交换机互相连接且没有 VLAN 隔离，它们通常形成同一个广播域。

### 8.2 为什么两台 `.226` 会冲突

假设两个机械臂控制器都为 `192.168.2.226`，并接入同一个二层广播域：

1. Orin 广播询问“谁是 `.226`”；
2. 两个控制器都可能回复不同 MAC；
3. ARP 表可能来回变化；
4. TCP 控制连接可能连错机器人或突然断开。

这不是 ROS 2 Domain ID 能解决的问题。

可靠方案按优先级排列：

1. 两台控制器位于完全分离的物理交换机/网卡链路；
2. 使用支持 VLAN 的交换机，将两套控制器放入不同 VLAN；
3. 使用不同网络命名空间或专用路由设备做严格隔离；
4. 如果厂商允许，修改控制器 IP 为不同地址。

仅靠在同一个交换机上增加一条 `/32` 路由，不能区分同一广播域内的两个 `.226`。

### 8.3 `/32` 地址和主机路由

`192.168.2.171/32` 表示只有这一个本机地址，不自动声明整个 `192.168.2.0/24` 都在该接口上。它常用于多网口同号段时，配合明确的目标主机路由：

```bash
# 示例；执行前必须先确认接口名和物理接线
sudo ip addr replace 192.168.2.171/32 dev <推理侧接口>
sudo ip route replace 192.168.2.110/32 dev <推理侧接口> src 192.168.2.171
```

`.110` 也需要知道返回 `.171` 的路径。如果双方直连或处于同一个隔离交换机，通常可添加对称主机路由：

```bash
# 在 4090 主机上的概念示例
sudo ip route replace 192.168.2.171/32 dev <连接新Orin的接口> src 192.168.2.110
```

临时 `ip addr` / `ip route` 命令重启后通常消失。确认方案正确后，再由管理员写入 NetworkManager、Netplan 或 systemd-networkd；不要在不知道连接名称时直接覆盖网络配置。

### 8.4 `net.ipv4.ip_forward=0` 是什么

```bash
sysctl net.ipv4.ip_forward
```

`net.ipv4.ip_forward=0` 表示 Orin 不充当 IPv4 路由器，不把一个网口收到的普通 IP 包转发到另一个网口。它有助于阻止两个本应隔离的网络被 Orin 桥接。

它不表示“除了 `.110` 以外都不回复”，也不是防火墙白名单。Orin 自己地址收到的请求仍可能回复。访问控制需使用 nftables/iptables 或交换机 VLAN。

## 9. 建议的多设备隔离拓扑

两台机器人控制器 IP 相同时，建议：

```text
控制网 A（物理/VLAN 隔离）
新 Orin 专用网口 A ── 独立交换机/VLAN A ── 新控制器 192.168.2.226

控制网 B（物理/VLAN 隔离）
旧主机专用网口 B ── 独立交换机/VLAN B ── 旧控制器 192.168.2.226

推理网
新 Orin 专用网口 C ── 推理交换机 ── 4090 主机 192.168.2.110
旧机器人推理口  ─────┘

操作网
Windows 192.168.2.130 ── 指定交换机/Orin 操作口 ── 新 Orin 192.168.2.170
```

推理网可以共享，因为 `.110` 地址唯一，且 4090 只提供模型服务。两个固定为 `.226` 的控制器不能共享同一未隔离二层网络。

## 10. 4090 主机基础配置

当前已知：

- 地址：`192.168.2.110`；
- 用户：`walle`；
- 两张 RTX 4090；
- 项目约定只使用物理 GPU 1；
- 4090 做推理，Orin 做 ROS 2 和机器人控制。

SSH 别名示例：

```sshconfig
Host lingbot-4090
    HostName 192.168.2.110
    User walle
    Port 22
    IdentityFile C:/Users/<Windows用户名>/.ssh/lingbot_4090_ed25519
    IdentitiesOnly yes
    ServerAliveInterval 30
    ServerAliveCountMax 3
```

这只是一份模板。`.110` 的公钥是否已经安装、专用密钥文件名和网口名仍需现场确认。

选择 GPU 1：

```bash
export CUDA_VISIBLE_DEVICES=1
nvidia-smi
```

注意：设置 `CUDA_VISIBLE_DEVICES=1` 后，程序内部通常会把这张物理 GPU 1 显示成逻辑 `cuda:0`，这是正常的。

## 11. 常见错误定位

### 11.1 `Connection timed out`

含义：TCP 22 在超时前没有建立连接。优先检查：

1. Orin 是否开机；
2. 网线/交换机链路灯；
3. Windows 以太网是否 `Up`；
4. Windows 是否仍为 `.130/24`；
5. `ping -S 192.168.2.130 192.168.2.170`；
6. 是否连到了另一套同地址设备或错误交换机；
7. 防火墙和 SSH 服务。

### 11.2 `Connection refused`

IP 已经到达目标，但 TCP 22 没有服务监听或被主动拒绝。在 Orin 本地检查：

```bash
systemctl status ssh
ss -lntp | grep ':22'
```

### 11.3 `Permission denied (publickey)`

网络和 SSH 服务正常，但密钥认证失败。检查：

```powershell
ssh -vvv -i "$env:USERPROFILE\.ssh\one_arm_teleop_ed25519" nvidia@192.168.2.170
```

在 Orin 检查：

```bash
ls -ld ~/.ssh
ls -l ~/.ssh/authorized_keys
```

目录应为 `700`，`authorized_keys` 应为 `600`，且属于 `nvidia` 用户。

### 11.4 `REMOTE HOST IDENTIFICATION HAS CHANGED`

可能是目标系统重装、IP 被另一台机器占用，也可能是安全风险。先核对当前设备身份和新指纹。确认确实更换了目标后才执行：

```powershell
ssh-keygen -R 192.168.2.170
ssh-keygen -R armstrong-orin
```

不要看到提示就直接删除旧指纹。

### 11.5 Ping 显示“来自本机：无法访问目标主机”

这通常不是“网速不够”，而是本机没有获得目标 MAC、链路断开或路由选错。查看：

```powershell
arp -a
route print -4
Get-NetAdapter
```

Orin 上查看：

```bash
ip neigh
ip route get 192.168.2.130
```

### 11.6 SSH 偶发重置

区分三类问题：

- `Connection reset`：连接已建立后被对端或中间网络复位；
- `Broken pipe`：一段时间没有响应，客户端判定断开；
- `timed out`：建立连接或持续通信时完全没有回复。

SSH config 中的：

```sshconfig
ServerAliveInterval 30
ServerAliveCountMax 3
```

可以帮助检测死连接，但不能修复坏网线、重复 IP、交换机环路或错误路由。

## 12. 每次开机的只读网络检查清单

Windows：

```powershell
Get-NetAdapter | Format-Table Name,Status,LinkSpeed -AutoSize
Get-NetIPAddress -AddressFamily IPv4 |
  Where-Object IPAddress -Like '192.168.*' |
  Format-Table InterfaceAlias,IPAddress,PrefixLength -AutoSize
ping -S 192.168.2.130 192.168.2.170
Test-NetConnection 192.168.2.170 -Port 22
ssh -o BatchMode=yes armstrong-orin "hostname; hostname -I"
```

Orin：

```bash
hostname
ip -br addr
ip route get 192.168.2.226
ping -c 4 192.168.2.226
ip route get 192.168.2.110
ping -c 4 192.168.2.110
sysctl net.ipv4.ip_forward
```

预期：

- 登录的确是 `nvidia-desktop`；
- Windows→Orin 明确使用 `.130 → .170`；
- Orin→控制器 `.226` 走新机器人专用控制口；
- Orin→4090 `.110` 走推理口；
- 两条路由不应误走到另一台机器人的控制网络；
- `ip_forward=0`；
- 没有运行任何机器人运动命令。

## 13. 当前仍需现场复核的内容

由于 2026-08-21 编写本文时新 Orin 未在线，以下内容不能写死：

- `.170` 当前具体对应 `eth0/eth4/eth6/eth7` 中哪一个；
- `.171/32` 是否已经持久化、是否仍在重启后存在；
- `.110` 的反向 `/32` 路由是否持久化；
- 新控制器 `.226` 与旧控制器 `.226` 是否始终处于物理/VLAN 隔离网络；
- 4090 交换机是普通二层交换机还是支持 VLAN 的管理型交换机；
- `.110` 当前连接新 Orin 的具体网口名称；
- 现场是否确实需要以太网默认网关 `192.168.2.1`。

复核时只需保存下面命令输出，不需要给机器人上电或使能：

```bash
hostname
ip -br link
ip -br addr
ip route
ip route get 192.168.2.226
ip route get 192.168.2.110
nmcli device status
nmcli -t -f NAME,DEVICE,TYPE connection show --active
sysctl net.ipv4.ip_forward
```

## 14. 最简记忆版

1. Windows 有线设为 `192.168.2.130/24`，机器人隔离网推荐不设网关和 DNS。
2. `ping -S 192.168.2.130 192.168.2.170` 验证确实走有线。
3. `ssh armstrong-orin` 登录 `nvidia@192.168.2.170`，主机名应为 `nvidia-desktop`。
4. 私钥留在 Windows，只有 `.pub` 公钥放入 Orin。
5. Orin 通过 `.226` 控制机械臂，通过 `.110` 请求 4090 推理。
6. `ROS_DOMAIN_ID` 只隔离 ROS 2，不解决重复 IP。
7. 两个 `.226` 必须物理隔离、VLAN 隔离或改 IP。
8. 交换机按 MAC 转发；普通交换机不会自动隔离同 IP 设备。
9. `/32` 主机路由只指定到一台主机的精确路径，不能修复同一广播域内的重复 IP。
10. 每次先用 `ip route get` 核对路径，再启动任何机器人程序。
