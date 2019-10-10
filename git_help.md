
# 配置

## ??
> git remote add origin https://username:password@github.com/zhyuzh3d/goWeb.git
> 
> git remote set-url origin https://username:password@github.com/zhyuzh3d/goWeb.git



## 每个机器都必须自报家门：你的名字和Email地址
> git config --global user.name "Your Name"
> 
> git config --global user.email "email@example.com"






# add/rm 和 commit

## 通过git init命令把这个目录变成Git可以管理的仓库：
> git init

## 用命令git add告诉Git，把文件添加/删除到仓库：  
> git add readme.txt
> 
> git rm test.txt

## 用命令git commit告诉Git，把文件提交到仓库：   
> git commit -m "wrote a readme file"


# 版本回退

## 查看日志
> git log

## 查看每次更改版本的版本号
> git reflog

## 回到版本
> git reset --hard HEAD^

## 回到前100个版本
> git reset --hard HEAD~100

## 根据版本号切换版本
> git reset --hard c071502

## 查看当前状态
> git status


# 撤销

## 用暂存区的文件覆盖工作区的文件
让这个文件回到最近一次git commit或git add时的状态
> git checkout -- readme.txt
>
> git restore readme.txt

## 用仓库的文件覆盖暂存区的文件
> git reset HEAD readme.txt




# 远程仓库

## 创建SSH Key
>  ssh-keygen -t rsa -C "youremail@example.com"

## 本地仓库关联 
> git remote add remote_repository_name git@github.com:52mama/HRT_Gaopeifeng.git


## 推送所有内容
把本地的master分支内容推送的远程新的master分支，还会把本地的master分支和远程的master分支关联起来
> git push -u origin master


## 推送最新内容
> git push  origin master


## 克隆库
> git clone git@github.com:user_name/repository_name.git

## !!!!!!!!!!!!!****
> git pull origin master ----allow-unrelated-histories


# 分支

## list 分支
> git branch

## 创建分支
> git branch dev

## 转换分支
> git switch dev

## 合并分支
把dev分支合并到当前分支
> git merge dev 

## 终止合并状态
> git merge --abort
> 
## 删除分支
> git branch -d dev 




