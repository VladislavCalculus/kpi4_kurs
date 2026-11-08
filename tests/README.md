# Перевірки

`ctest --test-dir build --output-on-failure` запускає всі Python-перевірки читача, CLI, місії та стану. Python-тести використовують лише стандартну бібліотеку; `kpi4_test_bridge` надає їм доступ до зібраного C++ модуля. Сценарії та очікувані результати містяться в `tests/python/`.

Після збирання можна запустити тільки цей набір:

```sh
ctest --test-dir build -R python_state --output-on-failure
```
