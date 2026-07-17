import pstats

p = pstats.Stats("server")
p.sort_stats("time").print_stats(0.1)  # 按照时间排序显示其内容，也可按照其他列显示
