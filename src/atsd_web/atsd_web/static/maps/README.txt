Подложки карт
=============

indoor.png   план помещения или карта занятости от slam_toolbox
outdoor.jpg  спутниковый снимок территории

Файлов может не быть — интерфейс тогда рисует метрическую сетку
и остаётся полностью рабочим.

Привязка задаётся в atsd_web/server.py, словарь DEFAULT_MAPS:

  meters_per_pixel  сколько метров в одном пикселе снимка
  origin            координаты левого нижнего угла картинки
                    в метрах во фрейме map

Как посчитать meters_per_pixel: найдите на снимке два объекта
с известным расстоянием между ними, померьте его в пикселях
в любом редакторе и поделите метры на пиксели.

Карту помещения после slam_toolbox сохраните так:
  ros2 run nav2_map_server map_saver_cli -f indoor
Получите indoor.pgm и indoor.yaml. В yaml лежит готовое
resolution — это и есть meters_per_pixel, а origin — ваш origin.
PGM переведите в PNG:
  convert indoor.pgm indoor.png
