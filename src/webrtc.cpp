/*
 * Includes code from https://github.com/centricular/gstwebrtc-demos/blob/master/sendrecv/gst/webrtc-sendrecv.c
 * up-to-date to commit e4b86bc4f151e35222aff1bf7e46cec016e7b0ee 2020-6-25
 */
 
#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#include <gst/app/app.h>

#define GST_USE_UNSTABLE_API
#include <gst/webrtc/webrtc.h>

/* For signalling */
#include <libsoup/soup.h>
#include <json-glib/json-glib.h>

#include <string.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <thread>
#include <std_msgs/String.h>

enum AppState {
  APP_STATE_UNKNOWN = 0,
  APP_STATE_ERROR = 1, /* generic error */
  SERVER_CONNECTING = 1000,
  SERVER_CONNECTION_ERROR,
  SERVER_CONNECTED, /* Ready to register */
  SERVER_REGISTERING = 2000,
  SERVER_REGISTRATION_ERROR,
  SERVER_REGISTERED, /* Ready to call a peer */
  SERVER_CLOSED, /* server connection closed by us or the server */
  PEER_CONNECTING = 3000,
  PEER_CONNECTION_ERROR,
  PEER_CONNECTED,
  PEER_CALL_NEGOTIATING = 4000,
  PEER_CALL_STARTED,
  PEER_CALL_STOPPING,
  PEER_CALL_STOPPED,
  PEER_CALL_ERROR,
};

namespace enc = sensor_msgs::image_encodings;


static GMainLoop *loop;
static GstElement *pipe1, *webrtc1, *appsrc_;
static GObject *send_channel, *receive_channel;

static SoupWebsocketConnection *ws_conn = nullptr;
static AppState app_state = APP_STATE_UNKNOWN;
static const gchar *peer_id = nullptr;
static const gchar *server_url = "wss://127.0.0.1:8443";
static gboolean disable_ssl = FALSE;

// added
std::unique_ptr<image_transport::ImageTransport> image_transport_;
image_transport::Subscriber image_sub_;
static GCancellable *serverConnection;
std::thread loop_thread_;
static int retries = 0;
static const int MAXRETRIES = 5;
gboolean serverConnected = FALSE;
gboolean serverConnectStarted = FALSE;
static const gchar *audio_device = nullptr;
static const gchar *video_width = nullptr; 
static const gchar *video_height = nullptr; 
static const gchar *video_bitrate = nullptr; 
static const gchar *turnserver_login = nullptr; 
static const gchar *turnserver_port = nullptr; 
static ros::Publisher webrtcstatus_pub;

static GOptionEntry entries[] =
{
  { "peer-id", 0, 0, G_OPTION_ARG_STRING, &peer_id, "String ID of the peer to connect to", "ID" },
  { "server", 0, 0, G_OPTION_ARG_STRING, &server_url, "Signalling server to connect to", "URL" },
  { "disable-ssl", 0, 0, G_OPTION_ARG_NONE, &disable_ssl, "Disable ssl", NULL },
  { "audio-device", 0, 0, G_OPTION_ARG_STRING, &audio_device, "String of the audio device number", "ID" },
  { "video-width", 0, 0, G_OPTION_ARG_STRING, &video_width, "video width pixels", NULL },
  { "video-height", 0, 0, G_OPTION_ARG_STRING, &video_height, "video height pixels", NULL },
  { "video-bitrate", 0, 0, G_OPTION_ARG_STRING, &video_bitrate, "target bitrate kbps", NULL },
  { "turnserver-port", 0, 0, G_OPTION_ARG_STRING, &turnserver_port, "TURN server port number", NULL },
  { "turnserver-login", 0, 0, G_OPTION_ARG_STRING, &turnserver_login, "TURN server user:pass", NULL },
  { NULL },
};

static gboolean
cleanup_and_quit_loop (const gchar * msg, AppState state)
{
  if (msg)
    g_printerr ("%s\n", msg);
  if (state > 0)
    app_state = state;

  if (ws_conn) {
    if (soup_websocket_connection_get_state (ws_conn) ==
        SOUP_WEBSOCKET_STATE_OPEN)
      /* This will call us again */
      soup_websocket_connection_close (ws_conn, 1000, "");
    else
      g_object_unref (ws_conn);
  }

  if (loop) {
    g_main_loop_quit (loop);
    //g_main_loop_unref(loop); // added - required?
    loop = NULL;
    g_print("g_main_loop quit\n");
  }

  /* To allow usage as a GSourceFunc */
  return G_SOURCE_REMOVE;
}

static gchar*
get_string_from_json_object (JsonObject * object)
{
  JsonNode *root;
  JsonGenerator *generator;
  gchar *text;

  /* Make it the root node */
  root = json_node_init_object (json_node_alloc (), object);
  generator = json_generator_new ();
  json_generator_set_root (generator, root);
  text = json_generator_to_data (generator, NULL);

  /* Release everything */
  g_object_unref (generator);
  json_node_free (root);
  return text;
}

static void
handle_media_stream (GstPad * pad, GstElement * pipe, const char * convert_name,
    const char * sink_name)
{
  GstPad *qpad;
  GstElement *q, *conv, *resample, *sink;
  GstPadLinkReturn ret;

  g_print ("Trying to handle stream with %s ! %s", convert_name, sink_name);

  q = gst_element_factory_make ("queue", NULL);
  g_assert_nonnull (q);
  conv = gst_element_factory_make (convert_name, NULL);
  g_assert_nonnull (conv);
  sink = gst_element_factory_make (sink_name, NULL);
  g_assert_nonnull (sink);

  if (g_strcmp0 (convert_name, "audioconvert") == 0) {
    /* Might also need to resample, so add it just in case.
     * Will be a no-op if it's not required. */
    resample = gst_element_factory_make ("audioresample", NULL);
    g_assert_nonnull (resample);
    gst_bin_add_many (GST_BIN (pipe), q, conv, resample, sink, NULL);
    gst_element_sync_state_with_parent (q);
    gst_element_sync_state_with_parent (conv);
    gst_element_sync_state_with_parent (resample);
    gst_element_sync_state_with_parent (sink);
    gst_element_link_many (q, conv, resample, sink, NULL);
  } else {
    gst_bin_add_many (GST_BIN (pipe), q, conv, sink, NULL);
    gst_element_sync_state_with_parent (q);
    gst_element_sync_state_with_parent (conv);
    gst_element_sync_state_with_parent (sink);
    gst_element_link_many (q, conv, sink, NULL);
  }

  qpad = gst_element_get_static_pad (q, "sink");

  ret = gst_pad_link (pad, qpad);
  g_assert_cmphex (ret, ==, GST_PAD_LINK_OK);
}

static void
on_incoming_decodebin_stream (GstElement * decodebin, GstPad * pad,
    GstElement * pipe)
{
  GstCaps *caps;
  const gchar *name;

  if (!gst_pad_has_current_caps (pad)) {
    g_printerr ("Pad '%s' has no caps, can't do anything, ignoring\n",
        GST_PAD_NAME (pad));
    return;
  }

  caps = gst_pad_get_current_caps (pad);
  name = gst_structure_get_name (gst_caps_get_structure (caps, 0));

  if (g_str_has_prefix (name, "video")) {
    handle_media_stream (pad, pipe, "videoconvert", "autovideosink");
  } else if (g_str_has_prefix (name, "audio")) {
    handle_media_stream (pad, pipe, "audioconvert", "autoaudiosink");
  } else {
    g_printerr ("Unknown pad %s, ignoring", GST_PAD_NAME (pad));
  }
}

static void
on_incoming_stream (GstElement * webrtc, GstPad * pad, GstElement * pipe)
{
  GstElement *decodebin;
  GstPad *sinkpad; 

  if (GST_PAD_DIRECTION (pad) != GST_PAD_SRC)
    return;

  decodebin = gst_element_factory_make ("decodebin", NULL);
  g_signal_connect (decodebin, "pad-added",
      G_CALLBACK (on_incoming_decodebin_stream), pipe);
  gst_bin_add (GST_BIN (pipe), decodebin);
  gst_element_sync_state_with_parent (decodebin);
  
  sinkpad = gst_element_get_static_pad (decodebin, "sink");
  gst_pad_link (pad, sinkpad);
  gst_object_unref (sinkpad);
}

static void
send_ice_candidate_message (GstElement * webrtc G_GNUC_UNUSED, guint mlineindex,
    gchar * candidate, gpointer user_data G_GNUC_UNUSED)
{
  gchar *text;
  JsonObject *ice, *msg;

  if (app_state < PEER_CALL_NEGOTIATING) {
    cleanup_and_quit_loop ("Can't send ICE, not in call", APP_STATE_ERROR);
    return;
  }

  ice = json_object_new ();
  json_object_set_string_member (ice, "candidate", candidate);
  json_object_set_int_member (ice, "sdpMLineIndex", mlineindex);
  msg = json_object_new ();
  json_object_set_object_member (msg, "ice", ice);
  text = get_string_from_json_object (msg);
  json_object_unref (msg);

  soup_websocket_connection_send_text (ws_conn, text);
  g_free (text);
}

static void
send_sdp_to_peer (GstWebRTCSessionDescription * desc)
{
  gchar *text;
  JsonObject *msg, *sdp;

  if (app_state < PEER_CALL_NEGOTIATING) {
    cleanup_and_quit_loop ("Can't send SDP to peer, not in call",
        APP_STATE_ERROR);
    return;
  }

  text = gst_sdp_message_as_text (desc->sdp);
  sdp = json_object_new ();

  if (desc->type == GST_WEBRTC_SDP_TYPE_OFFER) {
    g_print ("Sending offer:\n%s\n", text);
    json_object_set_string_member (sdp, "type", "offer");
  } else if (desc->type == GST_WEBRTC_SDP_TYPE_ANSWER) {
    g_print ("Sending answer:\n%s\n", text);
    json_object_set_string_member (sdp, "type", "answer");
  } else {
    g_assert_not_reached ();
  }

  json_object_set_string_member (sdp, "sdp", text);
  g_free (text);

  msg = json_object_new ();
  json_object_set_object_member (msg, "sdp", sdp);
  text = get_string_from_json_object (msg);
  json_object_unref (msg);

  soup_websocket_connection_send_text (ws_conn, text);
  g_free (text);
}

/* Offer created by our pipeline, to be sent to the peer */
static void
on_offer_created (GstPromise * promise, gpointer user_data)
{
  GstWebRTCSessionDescription *offer = NULL;
  const GstStructure *reply;

  g_assert_cmphex (app_state, ==, PEER_CALL_NEGOTIATING);

  g_assert_cmphex (gst_promise_wait(promise), ==, GST_PROMISE_RESULT_REPLIED);
  reply = gst_promise_get_reply (promise);
  gst_structure_get (reply, "offer",
      GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, NULL);
  gst_promise_unref (promise);

  promise = gst_promise_new ();
  g_signal_emit_by_name (webrtc1, "set-local-description", offer, promise);
  gst_promise_interrupt (promise);
  gst_promise_unref (promise);

  /* Send offer to peer */
  send_sdp_to_peer (offer);
  gst_webrtc_session_description_free (offer);
}

static void
on_negotiation_needed (GstElement * element, gpointer user_data)
{
  GstPromise *promise;

  app_state = PEER_CALL_NEGOTIATING;
  promise = gst_promise_new_with_change_func (on_offer_created, user_data, NULL);
  g_signal_emit_by_name (webrtc1, "create-offer", NULL, promise);
}

static void
on_ice_gathering_state_notify (GstElement * webrtcbin, GParamSpec * pspec,
    gpointer user_data)
{
  GstWebRTCICEGatheringState ice_gather_state;
  const gchar *new_state = "unknown";

  g_object_get (webrtcbin, "ice-gathering-state", &ice_gather_state, NULL);
  switch (ice_gather_state) {
    case GST_WEBRTC_ICE_GATHERING_STATE_NEW:
      new_state = "new";
      break;
    case GST_WEBRTC_ICE_GATHERING_STATE_GATHERING:
      new_state = "gathering";
      break;
    case GST_WEBRTC_ICE_GATHERING_STATE_COMPLETE:
      new_state = "complete";
      break;
  }
  g_print ("ICE gathering state changed to %s\n", new_state);
}


#define STUN_SERVER " stun-server=stun://stun.l.google.com:19302 "
#define RTP_CAPS_OPUS "application/x-rtp,media=audio,encoding-name=OPUS,payload="
#define RTP_CAPS_VP8 "application/x-rtp,media=video,encoding-name=VP8,payload="

static gboolean start_pipeline (void)
{
  GstStateChangeReturn ret;
  GError *error = NULL;

	if(!audio_device) {
		gchar *pl = g_strconcat ("webrtcbin bundle-policy=max-bundle name=sendrecv " STUN_SERVER
			"turn-server=turn://", turnserver_login, "@127.0.0.1:", turnserver_port, " "
			"videorate ! video/x-raw,width=", video_width, ",height=", video_height, ",framerate=15/1 ! "
			"videoconvert ! queue ! vp8enc deadline=1 target-bitrate=", video_bitrate, "000 ! rtpvp8pay ! "
			"queue ! " RTP_CAPS_VP8 "96 ! sendrecv. ", NULL);
			
		pipe1 =	gst_parse_launch (pl, &error);
		g_free(pl);
    }
    else {  
		gchar *pl = g_strconcat ("webrtcbin bundle-policy=max-bundle name=sendrecv " STUN_SERVER 
			"turn-server=turn://", turnserver_login, "@127.0.0.1:", turnserver_port, " "
			"videorate ! video/x-raw,width=", video_width, ",height=", video_height, ",framerate=15/1 ! "
			"videoconvert ! queue ! vp8enc deadline=1 target-bitrate=", video_bitrate, "000 ! rtpvp8pay ! "
			"queue ! " RTP_CAPS_VP8 "96 ! sendrecv. "
			"alsasrc device=hw:", audio_device, " ! audioconvert ! audioresample ! queue ! opusenc ! rtpopuspay ! "
			"queue ! " RTP_CAPS_OPUS "97 ! sendrecv. ", NULL);

		pipe1 =	gst_parse_launch (pl, &error);
		g_free(pl);
	}

	if (error) {
		g_printerr ("Failed to parse launch: %s\n", error->message);
		g_error_free (error);
		if (pipe1)  g_clear_object (&pipe1);
		return FALSE;
	}
	
  
  	// init appsrc
  	appsrc_ = gst_element_factory_make("appsrc", "source");
	if (appsrc_ == nullptr) {
		ROS_ERROR("GST: failed to create appsrc!");
		return FALSE;
	}

	gst_app_src_set_stream_type(GST_APP_SRC_CAST(appsrc_), GST_APP_STREAM_TYPE_STREAM);
	gst_app_src_set_latency(GST_APP_SRC_CAST(appsrc_), 0, -1);
	g_object_set(GST_OBJECT(appsrc_),
			"format", GST_FORMAT_TIME,
			"is-live", true,
			"max-bytes", 0,
			"do-timestamp", true,
			NULL);

	// find pipeline sink (where we may link appsrc)
	GstPad *inpad = gst_bin_find_unlinked_pad(GST_BIN(pipe1), GST_PAD_SINK);
	g_assert(inpad);

	GstElement *inelement = gst_pad_get_parent_element(inpad);
	g_assert(inelement);
	gst_object_unref(GST_OBJECT(inpad));
	ROS_INFO("GST: inelement: %s", gst_element_get_name(inelement));

	if (!gst_bin_add(GST_BIN(pipe1), appsrc_)) {
		ROS_ERROR("GST: gst_bin_add() failed!");
		gst_object_unref(GST_OBJECT(pipe1));
		gst_object_unref(GST_OBJECT(inelement));
		return FALSE;
	}

	if (!gst_element_link(appsrc_, inelement)) {
		ROS_ERROR("GST: cannot link %s -> %s",
		gst_element_get_name(appsrc_),
		gst_element_get_name(inelement));
		gst_object_unref(GST_OBJECT(pipe1));
		gst_object_unref(GST_OBJECT(inelement));
		return FALSE;
	}

	gst_object_unref(GST_OBJECT(inelement));	
	
  
  
	
	webrtc1 = gst_bin_get_by_name (GST_BIN (pipe1), "sendrecv");
	g_assert_nonnull (webrtc1);

	/* This is the gstwebrtc entry point where we create the offer and so on. It
	* will be called when the pipeline goes to PLAYING. */
	g_signal_connect (webrtc1, "on-negotiation-needed",
	  G_CALLBACK (on_negotiation_needed), NULL);
	/* We need to transmit this ICE candidate to the browser via the websockets
	* signalling server. Incoming ice candidates from the browser need to be
	* added by us too, see on_server_message() */
	g_signal_connect (webrtc1, "on-ice-candidate",
	  G_CALLBACK (send_ice_candidate_message), NULL);
	g_signal_connect (webrtc1, "notify::ice-gathering-state",
	  G_CALLBACK (on_ice_gathering_state_notify), NULL);

	gst_element_set_state (pipe1, GST_STATE_READY);

	/* Incoming streams will be exposed via this signal */
	g_signal_connect (webrtc1, "pad-added", G_CALLBACK (on_incoming_stream),
	  pipe1);
	/* Lifetime is the same as the pipeline itself */
	gst_object_unref (webrtc1);

	g_print ("Starting pipeline\n");
	ret = gst_element_set_state (GST_ELEMENT (pipe1), GST_STATE_PLAYING);
	if (ret == GST_STATE_CHANGE_FAILURE)
	goto err;
		
	return TRUE;

err:
	g_print ("ERROR starting pipeline\n");
	if (pipe1)
		g_clear_object (&pipe1);
	if (webrtc1)
		webrtc1 = NULL;
	return FALSE;
}

static gboolean
setup_call (void)
{
  gchar *msg;

  if (soup_websocket_connection_get_state (ws_conn) !=
      SOUP_WEBSOCKET_STATE_OPEN)
    return FALSE;

  if (!peer_id)
    return FALSE;

  g_print ("Setting up signalling server call with %s\n", peer_id);
  app_state = PEER_CONNECTING;
  msg = g_strdup_printf ("SESSION %s", peer_id);
  soup_websocket_connection_send_text (ws_conn, msg);
  g_free (msg);
  return TRUE;
}

static gboolean
register_with_server (void)
{
  gchar *hello;
  gint32 our_id;

  if (soup_websocket_connection_get_state (ws_conn) !=
      SOUP_WEBSOCKET_STATE_OPEN)
    return FALSE;

  our_id = g_random_int_range (10, 10000);
  g_print ("Registering id %i with server\n", our_id);
  app_state = SERVER_REGISTERING;

  /* Register with the server with a random integer id. Reply will be received
   * by on_server_message() */
  hello = g_strdup_printf ("HELLO %i", our_id);
  soup_websocket_connection_send_text (ws_conn, hello);
  g_free (hello);

  return TRUE;
}

static void
on_server_closed (SoupWebsocketConnection * conn G_GNUC_UNUSED,
    gpointer user_data G_GNUC_UNUSED)
{
	app_state = SERVER_CLOSED;
	// cleanup_and_quit_loop ("Server connection closed", APP_STATE_UNKNOWN);
	g_print ("Server connection closed, trying to continue anyway\n");

	std_msgs::String msg;
    msg.data = "disconnected";
    webrtcstatus_pub.publish(msg);
}

/* One mega message handler for our asynchronous calling mechanism */
static void
on_server_message (SoupWebsocketConnection * conn, SoupWebsocketDataType type,
    GBytes * message, gpointer user_data)
{
  gchar *text;

  switch (type) {
    case SOUP_WEBSOCKET_DATA_BINARY:
      g_printerr ("Received unknown binary message, ignoring\n");
      return;
    case SOUP_WEBSOCKET_DATA_TEXT: {
      gsize size;
      const gchar *data = reinterpret_cast<const gchar*>(g_bytes_get_data (message, &size));
      /* Convert to NULL-terminated string */
      text = g_strndup (data, size);
      break;
    }
    default:
      g_assert_not_reached ();
  }

  /* Server has accepted our registration, we are ready to send commands */
  if (g_strcmp0 (text, "HELLO") == 0) {
    if (app_state != SERVER_REGISTERING) {
      cleanup_and_quit_loop ("ERROR: Received HELLO when not registering",
          APP_STATE_ERROR);
      goto out;
    }
    app_state = SERVER_REGISTERED;
    g_print ("Registered with server\n");
    /* Ask signalling server to connect us with a specific peer */
    if (!setup_call ()) {
      cleanup_and_quit_loop ("ERROR: Failed to setup call", PEER_CALL_ERROR);
      goto out;
    }
  /* Call has been setup by the server, now we can start negotiation */
  } else if (g_strcmp0 (text, "SESSION_OK") == 0) {
    if (app_state != PEER_CONNECTING) {
      cleanup_and_quit_loop ("ERROR: Received SESSION_OK when not calling",
          PEER_CONNECTION_ERROR);
      goto out;
    }

    app_state = PEER_CONNECTED;
    /* Start negotiation (exchange SDP and ICE candidates) */
    if (!start_pipeline ())
      cleanup_and_quit_loop ("ERROR: failed to start pipeline",
          PEER_CALL_ERROR);
  /* Handle errors */
  } else if (g_str_has_prefix (text, "ERROR")) {
    switch (app_state) {
      case SERVER_CONNECTING:
        app_state = SERVER_CONNECTION_ERROR;
        break;
      case SERVER_REGISTERING:
        app_state = SERVER_REGISTRATION_ERROR;
        break;
      case PEER_CONNECTING:
        app_state = PEER_CONNECTION_ERROR;
        break;
      case PEER_CONNECTED:
      case PEER_CALL_NEGOTIATING:
        app_state = PEER_CALL_ERROR;
        break;
      default:
        app_state = APP_STATE_ERROR;
    }
    cleanup_and_quit_loop (text, APP_STATE_UNKNOWN);
  /* Look for JSON messages containing SDP and ICE candidates */
  } else {
    JsonNode *root;
    JsonObject *object, *child;
    JsonParser *parser = json_parser_new ();
    if (!json_parser_load_from_data (parser, text, -1, NULL)) {
      g_printerr ("Unknown message '%s', ignoring", text);
      g_object_unref (parser);
      goto out;
    }

    root = json_parser_get_root (parser);
    if (!JSON_NODE_HOLDS_OBJECT (root)) {
      g_printerr ("Unknown json message '%s', ignoring", text);
      g_object_unref (parser);
      goto out;
    }

    object = json_node_get_object (root);
    /* Check type of JSON message */
    if (json_object_has_member (object, "sdp")) {
      int ret;
      GstSDPMessage *sdp;
      const gchar *text, *sdptype;
      GstWebRTCSessionDescription *answer;

      g_assert_cmphex (app_state, ==, PEER_CALL_NEGOTIATING);

      child = json_object_get_object_member (object, "sdp");

      if (!json_object_has_member (child, "type")) {
        cleanup_and_quit_loop ("ERROR: received SDP without 'type'",
            PEER_CALL_ERROR);
        goto out;
      }

      sdptype = json_object_get_string_member (child, "type");
      /* In this example, we create the offer and receive one answer by default,
       * but it's possible to comment out the offer creation and wait for an offer
       * instead, so we handle either here.
       *
       * See tests/examples/webrtcbidirectional.c in gst-plugins-bad for another
       * example how to handle offers from peers and reply with answers using webrtcbin. */
      text = json_object_get_string_member (child, "sdp");
      ret = gst_sdp_message_new (&sdp);
      g_assert_cmphex (ret, ==, GST_SDP_OK);
      ret = gst_sdp_message_parse_buffer ((guint8 *) text, strlen (text), sdp);
      g_assert_cmphex (ret, ==, GST_SDP_OK);

      if (g_str_equal (sdptype, "answer")) {
        g_print ("Received answer:\n%s\n", text);
        answer = gst_webrtc_session_description_new (GST_WEBRTC_SDP_TYPE_ANSWER,
            sdp);
        g_assert_nonnull (answer);

        /* Set remote description on our pipeline */
        {
          GstPromise *promise = gst_promise_new ();
          g_signal_emit_by_name (webrtc1, "set-remote-description", answer,
              promise);
          gst_promise_interrupt (promise);
          gst_promise_unref (promise);
        }
        app_state = PEER_CALL_STARTED;
      } 
      
    } else if (json_object_has_member (object, "ice")) {
      const gchar *candidate;
      gint sdpmlineindex;

      child = json_object_get_object_member (object, "ice");
      candidate = json_object_get_string_member (child, "candidate");
      sdpmlineindex = json_object_get_int_member (child, "sdpMLineIndex");

      /* Add ice candidate sent by remote peer */
      g_signal_emit_by_name (webrtc1, "add-ice-candidate", sdpmlineindex,
          candidate);
    } else {
      g_printerr ("Ignoring unknown JSON message:\n%s\n", text);
    }
    g_object_unref (parser);
  }

out:
  g_free (text);
}

static void
on_server_connected (SoupSession * session, GAsyncResult * res,
    SoupMessage *msg)
{
	serverConnected = TRUE;
	
  GError *error = NULL;

  ws_conn = soup_session_websocket_connect_finish (session, res, &error);
  if (error) {
    cleanup_and_quit_loop (error->message, SERVER_CONNECTION_ERROR);
    g_error_free (error);
    return;
  }

  g_assert_nonnull (ws_conn);

  app_state = SERVER_CONNECTED;
  g_print ("Connected to signalling server\n");

  g_signal_connect (ws_conn, "closed", G_CALLBACK (on_server_closed), NULL);
  g_signal_connect (ws_conn, "message", G_CALLBACK (on_server_message), NULL);

  /* Register with the server so it knows about us and can accept commands */
  register_with_server ();
  
  	std_msgs::String rosmsg; 
    rosmsg.data = "connected";
    webrtcstatus_pub.publish(rosmsg);
}

/*
 * Connect to the signalling server. This is the entrypoint for everything else.
 */
static void
connect_to_websocket_server_async (void)
{
  SoupLogger *logger;
  SoupMessage *message;
  SoupSession *session;
  const char *https_aliases[] = {"wss", NULL};

  session = soup_session_new_with_options (SOUP_SESSION_SSL_STRICT, !disable_ssl,
      SOUP_SESSION_SSL_USE_SYSTEM_CA_FILE, TRUE,
      //SOUP_SESSION_SSL_CA_FILE, "/etc/ssl/certs/ca-bundle.crt",
      SOUP_SESSION_HTTPS_ALIASES, https_aliases, NULL);

  // logger = soup_logger_new (SOUP_LOGGER_LOG_BODY, -1);
  // soup_session_add_feature (session, SOUP_SESSION_FEATURE (logger));
  // g_object_unref (logger);

  message = soup_message_new (SOUP_METHOD_GET, server_url);

	gchar *msg = g_strconcat ("Connecting to server: ", server_url,"\n", NULL);
	g_print (msg);
	g_free(msg);
  
  serverConnection = g_cancellable_new ();

  /* Once connected, we will register */
  soup_session_websocket_connect_async (session, message, NULL, NULL, serverConnection,
      (GAsyncReadyCallback) on_server_connected, message);
  // ^^ non blocking, sometimes fails initial connect

  app_state = SERVER_CONNECTING;
}

static gboolean
check_plugins (void)
{
  int i;
  gboolean ret;
  GstPlugin *plugin;
  GstRegistry *registry;
  const gchar *needed[] = { "opus", "vpx", "nice", "webrtc", "dtls", "srtp",
      "rtpmanager", "videotestsrc", "audiotestsrc", NULL};

  registry = gst_registry_get ();
  ret = TRUE;
  for (i = 0; i < g_strv_length ((gchar **) needed); i++) {
    plugin = gst_registry_find_plugin (registry, needed[i]);
    if (!plugin) {
      g_print ("Required gstreamer plugin '%s' not found\n", needed[i]);
      ret = FALSE;
      continue;
    }
    gst_object_unref (plugin);
  }
  return ret;
}

GstCaps* gst_caps_new_from_image(const sensor_msgs::Image::ConstPtr &msg)
{
	// http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
	static const ros::M_string known_formats = {{
		{enc::RGB8, "RGB"},
		{enc::RGB16, "RGB16"},
		{enc::RGBA8, "RGBA"},
		{enc::RGBA16, "RGBA16"},
		{enc::BGR8, "BGR"},
		{enc::BGR16, "BGR16"},
		{enc::BGRA8, "BGRA"},
		{enc::BGRA16, "BGRA16"},
		{enc::MONO8, "GRAY8"},
		{enc::MONO16, "GRAY16_LE"},
	}};

	if (msg->is_bigendian) {
		ROS_ERROR("GST: big endian image format is not supported");
		return nullptr;
	}

	auto format = known_formats.find(msg->encoding);
	if (format == known_formats.end()) {
		ROS_ERROR("GST: image format '%s' unknown", msg->encoding.c_str());
		return nullptr;
	}

	return gst_caps_new_simple("video/x-raw",
			"format", G_TYPE_STRING, format->second.c_str(),
			"width", G_TYPE_INT, msg->width,
			"height", G_TYPE_INT, msg->height,
			"framerate", GST_TYPE_FRACTION, 0, 1,	// 0/1 = dynamic
			nullptr);
}

GstSample* gst_sample_new_from_image(const sensor_msgs::Image::ConstPtr &msg)
{
	auto buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
	g_assert(buffer);

	gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
	GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);

	auto caps = gst_caps_new_from_image(msg);
	if (caps == nullptr) {
		gst_object_unref(GST_OBJECT(buffer));
		return nullptr;
	}

	auto sample = gst_sample_new(buffer, caps, nullptr, nullptr);
	gst_buffer_unref(buffer);
	gst_caps_unref(caps);

	return sample;
}




void startLoopAndConnect() {
	serverConnectStarted = TRUE;

	loop = g_main_loop_new (NULL, FALSE);

	connect_to_websocket_server_async ();
	
	loop_thread_ = std::thread(
		[&]() {
			g_main_loop_run (loop);
			g_main_loop_unref (loop);
			g_print("loop_thread_ exit\n");
		});
	// loop_thread_.detach();
		
}

void image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
	// ROS_INFO("Image: %d x %d, stamp %f", msg->width, msg->height, msg->header.stamp.toSec());
	if (!serverConnectStarted) {
		startLoopAndConnect();
		return;
	}
	
	if (!pipe1) return;
	
	auto sample = gst_sample_new_from_image(msg);
	if (sample == nullptr)
		return;
		
	auto push_ret = gst_app_src_push_sample(GST_APP_SRC_CAST(appsrc_), sample);
	gst_sample_unref(sample);
}


int main(int argc, char **argv) 
{
	// added
	ros::init(argc, argv, "webrtc");
	ros::NodeHandle n;
	
	image_transport_.reset(new image_transport::ImageTransport(n));
	ROS_INFO("INIT webrtc");

	image_sub_ = image_transport_->subscribe("image_raw", 10, image_cb); 
	
  GOptionContext *context;
  GError *error = NULL;

  context = g_option_context_new ("- gstreamer webrtc sendrecv");
  g_option_context_add_main_entries (context, entries, NULL);
  g_option_context_add_group (context, gst_init_get_option_group ());
  if (!g_option_context_parse (context, &argc, &argv, &error)) {
    g_printerr ("Error initializing: %s\n", error->message);
    return -1;
  }

  if (!check_plugins ())
    return -1;


  /* Disable ssl when running a localhost server, because
   * it's probably a test server with a self-signed certificate */

    GstUri *uri = gst_uri_from_string (server_url);
    if (g_strcmp0 ("localhost", gst_uri_get_host (uri)) == 0 ||
        g_strcmp0 ("127.0.0.1", gst_uri_get_host (uri)) == 0)
      disable_ssl = true;
    gst_uri_unref (uri);

	double start = 0;
	
	webrtcstatus_pub = n.advertise<std_msgs::String>("webrtcstatus", 10);
	
	std_msgs::String msg; 
    msg.data = "waiting";
    webrtcstatus_pub.publish(msg);
	
	ros::Rate r(100); // 100 hz
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();

		if (loop == NULL && serverConnectStarted) break;
		
		if (serverConnectStarted && start==0) start = ros::Time::now().toSec();
		
		if (start !=0 && !serverConnected && ros::Time::now().toSec() - start > 2) {
			g_printerr("server connect failure, exit\n");
			return -1;
		}

	}
	
	// cleanup_and_quit_loop("ros shutdown", APP_STATE_UNKNOWN); // shutdown GMainLoop

  if (pipe1) {
    gst_element_set_state (GST_ELEMENT (pipe1), GST_STATE_NULL);
    g_print ("Pipeline stopped\n");
    gst_object_unref (pipe1);
  }

  return 0;
}
