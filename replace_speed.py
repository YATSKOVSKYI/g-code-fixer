from pathlib import Path

path = Path('g_code_fixer/gui.py')
text = path.read_text(encoding='utf-8')
old = '        self.speed_label.setText(f"��������: {value} ��/�")'
new = '        self.speed_label.setText(f"Скорость: {value} мм/с")'
if old not in text:
    raise SystemExit('target not found')
path.write_text(text.replace(old, new, 1), encoding='utf-8')
