## 1.git push -u master

```
git push -u origin master 
相当于
git branch --set-upstream-to=origin/master master//将远程仓库origin的master分支与本地仓库master分支关联
加
git push origin master
```

## 1.github本地创建仓库并推送

（1）在github上新建一个仓库，与本地文件夹同名

（2）本地项目文件夹打开终端，初始化

```
git init (默认创建master分支）
git init -b main(创建main分支）
```

（3）添加项目和commit

```
git add .
git commit
```

（4）将本地代码仓库跟远程仓库进行关联

```
git remote add origin git@github.com:gaows123/shenlan_opt.git
git remote -v //查看关联情况
```

（5）推送到远程仓库

```
git push origin -u master (第一次)
git push (之后)
```

## 2.git push -u master

```shell
git push -u origin master 
//相当于,将远程仓库origin的master分支与本地仓库master分支关联
git branch --set-upstream-to=origin/master master
//加上
git push origin master
```

## 3.git branch -m xxx aaa

修改分支名称