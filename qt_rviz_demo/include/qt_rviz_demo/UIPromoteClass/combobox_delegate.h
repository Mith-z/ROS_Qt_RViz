#ifndef COMBOBOX_DELEGATE_H
#define COMBOBOX_DELEGATE_H

#include <QApplication>
#include <QComboBox>
#include <QStandardItemModel>
#include <QStyledItemDelegate>
#include <QTreeView>

class ComboBoxDelegate : public QStyledItemDelegate {
public:
  ComboBoxDelegate(QObject *parent = nullptr) : QStyledItemDelegate(parent) {}

  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                        const QModelIndex &index) const override {
    // 创建 QComboBox 编辑器
    QComboBox *editor = new QComboBox(parent);
    editor->addItem("");
    return editor;
  }

  void setEditorData(QWidget *editor, const QModelIndex &index) const override {
    // 设置 QComboBox 编辑器的当前值
    QString value = index.model()->data(index, Qt::EditRole).toString();
    QComboBox *comboBox = static_cast<QComboBox *>(editor);
    comboBox->setCurrentText(value);

    qDebug() << "edit";
  }

  void setModelData(QWidget *editor, QAbstractItemModel *model,
                    const QModelIndex &index) const override {
    // 更新模型数据为 QComboBox 编辑器的当前值
    QComboBox *comboBox = static_cast<QComboBox *>(editor);
    QString value = comboBox->currentText();
    model->setData(index, value, Qt::EditRole);

    qDebug() << "set";
  }

  void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option,
                            const QModelIndex &index) const override {
    // 更新 QComboBox 编辑器的位置和大小
    editor->setGeometry(option.rect);
  }
};

#endif // COMBOBOX_DELEGATE_H
