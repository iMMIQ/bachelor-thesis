# 主算法

> input: Rectangles{LL(左下点), LR(右下点), UR(右上点)}, start(起点), end(终点)

> output: path(路径), distanse(距离)

```
dp[][] := {INF ... INF, ... , INF ... INF}
```

dp[i][0]表示从可通过线段最低点离开矩形i的路径长度，dp[i][1]表示表示从可通过线段最高点离开矩形i的路径长度。

```
FOR i in (0, n - 1):
    L[i] := max({Rectangle[i].LL.y, Rectangle[i].LL.z}, {{Rectangle[i + 1].LL.y, Rectangle[i + 1].LL.z}}
    R[i] := min({Rectangle[i].UR.y, Rectangle[i].UR.z}, {{Rectangle[i + 1].UR.y, Rectangle[i + 1].UR.z}}
END FOR
```

这里向后看一个矩形，窄化判断范围

```
left_slope := -INF
right_slope := -INF
distanse := INF
path := {}
start_rectangle := find_start_rectangle()
end_rectangle := find_end_rectangle()
```

初始化

```
FOR i in range(start_rectangle, end_rectangle):
    IF i == start_rectangle:
        dp[i][0] := distance(start, {plane[i].UR.x, L[i]})
        dp[i][1] := distance(start, {plane[i].UR.x, R[i]})
        IF start.y < plane[i].LL.y || start.y > plane[i].UR.y:
            swap(left_slope, right_slope)
            BREAK
        END IF
    END IF
```

dp起始条件，以及起点纵坐标需要翻转的情况
