# Git 实操指南 — 基于 ranger_nav 的真实工作流

本指南用 ranger_nav 项目这次的实际操作过程，逐步讲解 Git 核心用法。面向没用过 Git 的 ROS 开发者。

---

## 1. 核心概念（先看这个）

Git 是分布式版本控制系统。三个关键区域：

```
工作区          暂存区          本地仓库          远程仓库
(你的文件)  →  (git add)  →  (git commit)  →  (git push)
  /home/...      .git/index       .git/objects      github.com
```

| 操作 | 含义 |
|------|------|
| `git add` | 告诉 Git "这些文件的改动我要记录" |
| `git commit` | 正式拍快照，附带一个描述信息 |
| `git push` | 把本地快照同步到 GitHub |
| `git pull` | 从 GitHub 拉取别人的更新 |
| `git fetch` | 只下载远程更新，不合并 |

**一句话**：add → commit → push 就是把你的修改安全上传到 GitHub 的完整流程。

---

## 2. 初始化仓库

### 场景：你有一个功能包目录，想用 Git 管理并上传到 GitHub

```bash
cd /home/robot/ranger_nav_ws/src/ranger_nav

# 初始化为 Git 仓库（只执行一次）
git init

# 查看状态：哪些文件改了、哪些还没跟踪
git status
```

`git init` 会在当前目录创建 `.git` 隐藏文件夹，所有版本历史都存在里面。

### .gitignore：告诉 Git 忽略哪些文件

```bash
# 创建 .gitignore（只执行一次）
cat > .gitignore << 'EOF'
build/
install/
log/
__pycache__/
*.pyc
.vscode/
.idea/
EOF
```

**为什么要忽略这些？**
- `build/` 和 `install/` 是 `colcon build` 生成的，每台机器自己编译就行
- `__pycache__/` 是 Python 字节码，跟项目无关
- `.vscode/` 是个人 IDE 配置，别人不需要

---

## 3. 第一次提交

```bash
# 添加所有文件（除了 .gitignore 里写的）
git add .

# 查看将要提交什么
git status

# 创建第一个快照
git commit -m "ranger_nav: initial commit"
```

**commit message 怎么写？**
- 第一行简短总结（< 50 字），英文或中文都行
- 多人协作建议英文：`ranger_nav: add SLAM config and launch files`
- 个人项目中文就行：`添加 pointcloud_to_laserscan 高度滤波参数调优`

---

## 4. 关联远程仓库

```bash
# 添加远程仓库地址（只执行一次）
git remote add origin git@github.com:Vitashelton/ranger_nav.git

# 查看远程地址
git remote -v
# 输出：
# origin  git@github.com:Vitashelton/ranger_nav.git (fetch)
# origin  git@github.com:Vitashelton/ranger_nav.git (push)
```

- `origin` 是远程仓库的别名，约定俗称的叫法
- `git@github.com:...` 是 SSH 地址（推荐，不用输密码）
- `https://github.com/...` 是 HTTPS 地址（需要 token）

### SSH 密钥配置（一次性）

```bash
# 生成 SSH 密钥（如果还没有）
ssh-keygen -t ed25519 -C "your_email@example.com"

# 查看公钥，复制到 GitHub Settings → SSH Keys
cat ~/.ssh/id_ed25519.pub

# 测试连接
ssh -T git@github.com
# 输出：Hi Vitashelton! You've successfully authenticated
```

---

## 5. 推送到 GitHub

### 如果 GitHub 上是空仓库

```bash
git push -u origin main
```

`-u` 是 `--set-upstream` 的缩写，让本地 `main` 分支记住它对应远程的 `origin/main`。设置一次后，以后直接 `git push` 就行。

### 如果 GitHub 上已有旧代码（我们这次遇到的情况）

```bash
# 第一步：拉取远程历史
git fetch origin

# 第二步：把本地改动放到远程历史之后
git rebase origin/main

# 如果出现冲突（文件同时被两边改了），手动解决：
#   冲突文件里会有 <<<<<<< ======= >>>>>>> 标记
#   编辑文件，保留你想要的版本，删掉标记
#   然后：
git add .
git rebase --continue

# 第三步：推送
git push origin main
```

---

## 6. 日常开发流程（以后每次改代码）

### 标准四步

```bash
# 1. 看看改了哪些文件
git status

# 2. 添加要提交的改动
git add config/pointcloud_to_laserscan.yaml    # 只加这个文件
# 或
git add .                                       # 加所有改动

# 3. 提交
git commit -m "调优：降低 min_height 到 0.3m 以捕获近场特征"

# 4. 推送到 GitHub
git push
```

### 常用变体

```bash
# 看具体改了什么内容
git diff                    # 未暂存的改动
git diff --staged           # 已暂存、等提交的改动

# 查看提交历史
git log --oneline -10       # 最近 10 条，一行一条
git log --oneline --graph   # 带分支图

# 撤销工作区的改动（放弃修改，回到上次提交的状态）
git restore <文件名>

# 撤销 git add（从暂存区退回到工作区）
git restore --staged <文件名>

# 修改最近一次 commit message
git commit --amend -m "新的commit message"
```

---

## 7. Jetson 上拉取更新

当你在这台 PC 上改完代码、推送到 GitHub 后，在 Jetson 上同步：

```bash
cd /home/robot/ranger_nav_ws/src/ranger_nav

# 拉取最新代码
git pull origin main

# 重新编译
cd /home/robot/ranger_nav_ws
colcon build --symlink-install
```

如果 Jetson 上还没克隆过：

```bash
cd /home/robot/ranger_nav_ws/src
git clone git@github.com:Vitashelton/ranger_nav.git
```

---

## 8. 分支管理（进阶）

当你同时维护 "稳定版" 和 "开发版"：

```bash
# 创建并切换到新分支
git checkout -b dev

# 在 dev 分支上开发、提交、推送
git add .
git commit -m "实验性功能"
git push -u origin dev

# 切回 main 分支
git checkout main

# 把 dev 的改动合并到 main
git merge dev

# 删除不需要的分支
git branch -d dev
```

---

## 9. 冲突解决速查

当 `git rebase` 或 `git merge` 报告冲突时：

```
<<<<<<< HEAD           ← 当前分支的版本
min_height: 0.05
=======
min_height: 0.3        ← 你想应用的新版本
>>>>>>> your-commit
```

**步骤：**
1. 用编辑器打开冲突文件
2. 删除 `<<<<<<<`、`=======`、`>>>>>>>` 这三行标记
3. 保留你想要的内容（比如保留 `0.3`）
4. `git add <文件>` 标记为已解决
5. `git rebase --continue` 继续

**快捷技巧（确定要全部用自己的版本时）：**

```bash
# 变基时，全部接受自己的改动
git checkout --theirs <文件>
git add <文件>
git rebase --continue

# 合并时，全部接受自己的改动
git checkout --ours <文件>
git add <文件>
git merge --continue
```

---

## 10. 本次操作的完整复盘

这次对 ranger_nav 做了什么：

```
场景：PC 上有个 ranger_nav 目录（没被 Git 管理），
      GitHub 上已经有个旧的 ranger_nav 仓库。

步骤                        Git 命令                       效果
──────────────────────────────────────────────────────────────────
1. 初始化仓库               git init                       创建 .git 目录
2. 创建 .gitignore          git add .gitignore             告诉 Git 忽略 build/ 等
3. 添加所有文件             git add .                      暂存所有代码
4. 本地提交                 git commit -m "..."            拍快照
5. 关联远程                 git remote add origin ...      指向 GitHub 仓库
6. 拉取远程历史             git fetch origin              看到远程有 main 分支
7. 分支改名                 git branch -m master main      统一分支名
8. 变基                     git rebase origin/main         把本地提交接到远程后面
   - 有冲突 → 手动解决      git checkout --theirs ...      保留自己的新版本
   - 标记解决               git add .
   - 继续                   GIT_EDITOR=true git rebase --continue
9. 推送                     git push origin main           上传到 GitHub
```

**关键理解：**
- **变基 (rebase)** = 把自己的提交"接到"远程最新提交的后面，保持历史是一条线
- **冲突** = 两个版本都改了同一行，Git 不知道该用哪个，需要你决定
- **`--theirs` vs `--ours`**：变基时是反的 —— `--theirs` 指的是你正在应用的那个提交（你要推的改动）

---

## 11. 速查表

| 我想做什么 | 命令 |
|-----------|------|
| 看状态 | `git status` |
| 看改了什么 | `git diff` |
| 加文件 | `git add <file>` 或 `git add .` |
| 提交 | `git commit -m "消息"` |
| 推送 | `git push` |
| 拉取 | `git pull origin main` |
| 看历史 | `git log --oneline` |
| 放弃改动 | `git restore <file>` |
| 放弃暂存 | `git restore --staged <file>` |
| 改 commit 消息 | `git commit --amend -m "新消息"` |
| 创建分支 | `git checkout -b <分支名>` |
| 切换分支 | `git checkout <分支名>` |
| 合并分支 | `git merge <分支名>` |
