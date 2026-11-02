# M01 — ручна перевірка читалки MAVLink

Прогін: 2026-11-02 13:27 Europe/Kiev (`+02:00`). Базовий коміт перед
прогоном: `1ff74de97cdbee4d2c9f54a025191e680f9eea5f`.
Робочий каталог: `/Users/admin/kpi3/kpi4/src`. Seed: не застосовується;
використано фіксовані бінарні фікстури. Версії системи, компілятора й CMake,
розміри та SHA-256 входів записані в [environment.txt](environment.txt).

## Команди й виходи

Точні команди та коди завершення є в [checks.tsv](checks.tsv) і
[commands.tsv](commands.tsv). Перед прогонами виконано `cmake -S . -B build`,
`cmake --build build`, `ctest --test-dir build --output-on-failure` та компіляцію
локального [двоканального probe](two_source_probe.cpp). Кожна команда завершилася
кодом 0; CTest — 12/12. Повний вивід перевірок збережено в `configure.*.txt`,
`build.*.txt`, `ctest.*.txt` і `compile_probe.*.txt` у цьому каталозі.

Основний `kpi4_cli` приймає лише один потік і позначає його `Unspecified`.
Для двох джерел `m01_two_source_probe` читає по одному байту поперемінно з
двох файлів і передає їх тому самому `FrameReader` як `GroundStation` і
`ControllerX`. Це відтворюваний ПК-прогін логічних каналів; фізичні GS і
контролер не підключалися.

| Сценарій та вхід | Очікувалося до запуску | Фактично | Повний stdout / stderr |
| --- | --- | --- | --- |
| `valid`: `heartbeat_v2.bin`, 21 байт, `--chunk-size 5` | Один MAVLink 2 `HEARTBEAT`; accepted=1, rejected=0, incomplete=0. | Один рядок `frame`, 5 порцій, accepted=1, rejected=0, incomplete=0; exit=0. | [stdout](valid.stdout.txt) / [stderr](valid.stderr.txt) |
| `damaged`: `recovery_stream.bin`, 86 байтів, `--chunk-size 5` | Два валідні кадри після шуму/пошкоджень; accepted=2, rejected=2, bad_crc=2, incomplete=0. | Два рядки `frame`, 18 порцій, accepted=2, rejected=2, bad_crc=2, incomplete=0; exit=0. | [stdout](damaged.stdout.txt) / [stderr](damaged.stderr.txt) |
| `two_sources`: GS=`heartbeat_v1.bin` (17 байтів), controller=`heartbeat_v2.bin` (21 байт) | По одному валідному кадру й нуль помилок у кожному каналі; правильні мітки походження. | `source=gs` має MAVLink 1, accepted=1; `source=controller` має MAVLink 2, accepted=1; обидва rejected=0, incomplete=0; exit=0. | [stdout](two_sources.stdout.txt) / [stderr](two_sources.stderr.txt) |
| `isolated_error`: GS=`heartbeat_v2_bad_crc.bin` (21 байт), controller=`heartbeat_v2.bin` (21 байт) | GS rejected=1 з bad_crc; controller accepted=1, rejected=0. | `source=gs rejected=bad_crc`; GS accepted=0/rejected=1; controller accepted=1/rejected=0; exit=0. | [stdout](isolated_error.stdout.txt) / [stderr](isolated_error.stderr.txt) |

Усі файли stderr порожні; їх збережено для повноти журналу. Час `frame time`
у stdout — UTC-час локального читання останнього байта, не час запису фікстури.

## Висновок

У цих чотирьох ПК-сценаріях читалка приймає повні кадри з коректним CRC,
повідомляє помилки й зберігає окремий стан двох логічних каналів. Дефектів у
перевірених сценаріях не виявлено. Перевірка не доводить сумісність із реальною
Ground Station або польотним контролером. Втрата байтів раніше останнього CRC
байта може приховати наступний кадр; це задокументоване обмеження парсера.
