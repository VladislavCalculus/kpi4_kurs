# Перевірки

`ctest --test-dir build --output-on-failure` запускає Python-перевірки читача та CLI і перевірки інших модулів. Python-тести використовують лише стандартну бібліотеку; `kpi4_test_bridge` надає їм доступ до зібраного C++ модуля. Сценарії та очікувані результати містяться в `tests/python/test_reader.py`.

Після збирання можна запустити тільки цей набір:

```sh
ctest --test-dir build -R python_reader --output-on-failure
```
