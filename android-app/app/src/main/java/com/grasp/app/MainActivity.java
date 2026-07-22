package com.grasp.app;

import android.annotation.SuppressLint;
import android.graphics.Bitmap;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.os.Bundle;
import android.view.KeyEvent;
import android.view.View;
import android.webkit.WebChromeClient;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.ProgressBar;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {

    private WebView webView;
    private ProgressBar progressBar;

    // ========== 修改这里的网址 ==========
    private static final String HOME_URL = "https://rzza.dxtxl.com:8083/";

    @SuppressLint("SetJavaScriptEnabled")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        webView = findViewById(R.id.webView);
        progressBar = findViewById(R.id.progressBar);

        setupWebView();

        // 检查网络并加载
        if (isNetworkAvailable()) {
            webView.loadUrl(HOME_URL);
        } else {
            Toast.makeText(this, "网络不可用，请检查网络连接", Toast.LENGTH_LONG).show();
            webView.loadUrl("file:///android_asset/offline.html");
        }
    }

    @SuppressLint("SetJavaScriptEnabled")
    private void setupWebView() {
        WebSettings settings = webView.getSettings();

        // 启用 JavaScript
        settings.setJavaScriptEnabled(true);

        // 启用 DOM Storage（localStorage / sessionStorage）
        settings.setDomStorageEnabled(true);

        // 启用缓存
        settings.setCacheMode(WebSettings.LOAD_DEFAULT);

        // 自适应屏幕
        settings.setUseWideViewPort(true);
        settings.setLoadWithOverviewMode(true);

        // 缩放
        settings.setSupportZoom(true);
        settings.setBuiltInZoomControls(true);
        settings.setDisplayZoomControls(false);

        // 允许混合内容（HTTP + HTTPS）
        settings.setMixedContentMode(WebSettings.MIXED_CONTENT_ALWAYS_ALLOW);

        // 允许文件访问
        settings.setAllowFileAccess(true);

        // 设置 User-Agent（伪装成 Chrome）
        settings.setUserAgentString(settings.getUserAgentString() + " GraspApp/1.0");

        // WebViewClient：在 WebView 内部打开链接
        webView.setWebViewClient(new WebViewClient() {
            @Override
            public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) {
                view.loadUrl(request.getUrl().toString());
                return true;
            }

            @Override
            public void onPageStarted(WebView view, String url, Bitmap favicon) {
                progressBar.setVisibility(View.VISIBLE);
            }

            @Override
            public void onPageFinished(WebView view, String url) {
                progressBar.setVisibility(View.GONE);
            }

            @Override
            public void onReceivedError(WebView view, WebResourceRequest request,
                                        WebResourceError error) {
                progressBar.setVisibility(View.GONE);
                if (request.isForMainFrame()) {
                    Toast.makeText(MainActivity.this,
                            "加载失败: " + error.getDescription(),
                            Toast.LENGTH_SHORT).show();
                }
            }
        });

        // WebChromeClient：处理进度、标题等
        webView.setWebChromeClient(new WebChromeClient() {
            @Override
            public void onProgressChanged(WebView view, int newProgress) {
                if (newProgress >= 100) {
                    progressBar.setVisibility(View.GONE);
                }
            }
        });
    }

    // 返回键处理：优先返回网页上一页
    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (keyCode == KeyEvent.KEYCODE_BACK && webView.canGoBack()) {
            webView.goBack();
            return true;
        }
        return super.onKeyDown(keyCode, event);
    }

    // 检查网络
    private boolean isNetworkAvailable() {
        ConnectivityManager cm = (ConnectivityManager) getSystemService(CONNECTIVITY_SERVICE);
        NetworkInfo info = cm.getActiveNetworkInfo();
        return info != null && info.isConnected();
    }
}
