#ifndef MULTISELECT_COMBOBOX_H
#define MULTISELECT_COMBOBOX_H

#include <QObject>
#include <QtWidgets>

class MultiSelectComboBox : public QComboBox {
public:
  MultiSelectComboBox(QWidget *parent = nullptr);
  ~MultiSelectComboBox() override;
  void addItem(const QString &text, const QVariant &userData = QVariant());
  void addItem(const QIcon &icon, const QString &text,
               const QVariant &userData = QVariant());
  void addItems(const QStringList &texts);
  QStringList currentText();
  QList<int> currentIndex();
  void uncheckAll();

  QStandardItemModel *GetSelectModel() { return selectModel; }

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;
  void showPopup() override;
  void hidePopup() override;

private:
  bool isPermitHidePopup;
  QListView *selectItemView;
  QStandardItemModel *selectModel;
  QListView *popupView;
  QStandardItemModel *popupModel;
  void selectItemViewPress(QPoint pos);
};

#endif // MULTISELECT_COMBOBOX_H
