# Dockerのセットアップ

ここではDockerのインストール方法とDockerイメージの構築方法を説明します。  
本編ではROSの環境をDockerを使って構築します。  
一般的にはROSを手動でインストールすることもありますが、今回はDockerを使ってROSの環境構築についてはできる限り手順を省略しています。

## Dockerのインストール

ドキュメントを参考にしながらDockerをインストールします。  
インストール手順は変わる可能性がありますので公式から案内される資料を参照してください。

* 公式ドキュメント（英語）
    * https://docs.docker.com/desktop/windows/install/
* 有志による非公式ドキュメント（日本語）
    * https://qiita.com/zaki-lknr/items/db99909ba1eb27803456

※ 2021年9月現在、WindowsでDockerを使用する際に必要なDocker Desktopは一定の条件を満たしている場合を除き、2022年2月から有料化が予定されているとのお知らせが掲載されています。詳しくは本資料の「参考情報」を参照してください。


## 参考情報

### Docker Desktopの料金プランについて

WindowsでDockerを使用する際に必要なDocker Desktopは一定の条件を満たしている場合を除き、2022年2月から有料化が予定されているとのことです（2021年9月現在）。

> Docker Desktopが有料化へ、ただし250人未満かつ年間売り上げ1000万ドル（約11億円）未満の組織や個人やオープンソースプロジェクトでは引き続き無料で利用可能
> https://www.publickey1.jp/blog/21/docker_desktop250100011.html

* 公式のお知らせ（英語）
    * https://www.docker.com/blog/updating-product-subscriptions/

> Docker Desktop for Linuxを開発中とDocker社が表明。有料化の発表が好評だったとして機能強化など加速
> https://www.publickey1.jp/blog/21/docker_desktop_for_linuxdocker.html

* 公式のお知らせ（英語）
    * https://www.docker.com/blog/accelerating-new-features-in-docker-desktop/

---

* [目次](./intro2.md)
* [INTRO2](./intro2.md)
* [STEP0](./step0.md)